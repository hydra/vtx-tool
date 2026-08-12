//! Routes `log` crate records into per-port message buffers, based on
//! `record.target()`. Use `target: "vtx"` / `target: "meter"` on log
//! calls to route them into the corresponding port's panel; anything
//! else goes to a general bucket.
//!
//! Usage: call `logging::init()` once at startup (before spawning the
//! worker thread), then use `log::error!(target: "vtx", "...")` /
//! `log::debug!(target: "meter", "...")` etc. throughout.

use log::{Level, LevelFilter, Log, Metadata, Record};
use std::collections::VecDeque;
use std::sync::Mutex;
use std::time::Instant;

pub const MAX_MESSAGES: usize = 100;

#[derive(Clone)]
pub struct LogEntry {
    pub level: Level,
    pub text: String,
    pub at: Instant,
}

#[derive(Default)]
pub struct PortLog {
    pub messages: VecDeque<LogEntry>,
}

impl PortLog {
    fn push(&mut self, entry: LogEntry) {
        self.messages.push_back(entry);
        while self.messages.len() > MAX_MESSAGES {
            self.messages.pop_front();
        }
    }
}

/// Global, process-wide log buckets. `Mutex`-protected so both the
/// worker thread (writer, via log:: macros) and the UI thread (reader,
/// rendering the panels) can access them safely.
pub struct SharedLogs {
    pub vtx: Mutex<PortLog>,
    pub meter: Mutex<PortLog>,
    pub general: Mutex<PortLog>,
}

impl SharedLogs {
    fn new() -> Self {
        Self {
            vtx: Mutex::new(PortLog::default()),
            meter: Mutex::new(PortLog::default()),
            general: Mutex::new(PortLog::default()),
        }
    }
}

struct BufferLogger {
    logs: &'static SharedLogs,
}

impl Log for BufferLogger {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        if !self.enabled(record.metadata()) {
            return;
        }
        let entry = LogEntry {
            level: record.level(),
            text: format!("{}", record.args()),
            at: Instant::now(),
        };
        let bucket = match record.target() {
            "vtx" => &self.logs.vtx,
            "meter" => &self.logs.meter,
            _ => &self.logs.general,
        };
        bucket.lock().unwrap().push(entry);
    }

    fn flush(&self) {}
}

/// Installs the global logger and returns a handle to the shared log
/// buffers for the UI to read from. Call once, before spawning any
/// threads that log.
pub fn init(max_level: LevelFilter) -> &'static SharedLogs {
    let logs: &'static SharedLogs = Box::leak(Box::new(SharedLogs::new()));
    let logger: &'static BufferLogger = Box::leak(Box::new(BufferLogger { logs }));
    log::set_logger(logger).expect("logger already initialized");
    log::set_max_level(max_level);
    logs
}

fn level_color(ui: &eframe::egui::Ui, level: Level) -> eframe::egui::Color32 {
    use eframe::egui::Color32;
    match level {
        Level::Error => Color32::from_rgb(220, 80, 80),
        Level::Warn => Color32::from_rgb(220, 180, 60),
        Level::Debug | Level::Trace => Color32::GRAY,
        Level::Info => ui.visuals().text_color(),
    }
}

/// Renders one port's scrollable log panel. `title` must be unique
/// across panels shown in the same frame (used as the egui id source for
/// the scroll area/grid).
pub fn show_panel(ui: &mut eframe::egui::Ui, title: &str, port_log: &PortLog) {
    use eframe::egui;

    ui.vertical(|ui| {
        ui.horizontal(|ui| {
            ui.strong(title);
            ui.label(format!("{} message(s)", port_log.messages.len()));
        });
        egui::ScrollArea::vertical()
            .id_salt(title)
            .auto_shrink([false, false])
            .stick_to_bottom(true)
            .show(ui, |ui| {
                egui::Grid::new(title).num_columns(1).striped(true).show(ui, |ui| {
                    for entry in port_log.messages.iter() {
                        let color = level_color(ui, entry.level);
                        ui.colored_label(color, format!("[{:>5}] {}", entry.level, entry.text));
                        ui.end_row();
                    }
                });
            });
    });
}
