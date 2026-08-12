//! Routes `log` crate records into per-port message buffers, based on
//! `record.target()`. Use `target: "vtx"` / `target: "meter"` on log
//! calls to route them into the corresponding port's panel; anything
//! else goes to a general bucket.
//!
//! ALSO forwards every record to a wrapped `env_logger::Logger`, so
//! everything additionally prints to the console gated by the RUST_LOG
//! environment variable, independent of the UI panels' own threshold.
//! Both live in the one logger installed via `log::set_logger()` --
//! that can only happen once process-wide, so this can't just run
//! `env_logger::init()` next to a separate custom logger; env_logger's
//! `Logger` type is used directly (built, not `.init()`-installed) and
//! delegated to from here instead.
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
    /// Threshold for the UI panels specifically (from --log-level),
    /// independent of env_logger's own RUST_LOG-driven threshold below.
    ui_max_level: LevelFilter,
    /// Not installed globally itself (that's `set_logger()` below, once,
    /// on this outer BufferLogger) -- just built and delegated to, so it
    /// applies its own RUST_LOG filtering and console formatting/writing
    /// exactly as it would standalone.
    console: env_logger::Logger,
}

impl Log for BufferLogger {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        // Permissive here -- see log() below, where the UI-panel and
        // console sinks are gated independently against their own
        // (different) thresholds.
        true
    }

    fn log(&self, record: &Record) {
        // Console, gated by RUST_LOG via the wrapped env_logger::Logger.
        // `Log::log()`'s contract expects the caller to check `enabled()`
        // first (env_logger doesn't re-check internally), so that's done
        // explicitly here rather than assumed.
        if self.console.enabled(record.metadata()) {
            self.console.log(record);
        }

        // UI panel buffers, gated by ui_max_level (--log-level).
        if record.level() <= self.ui_max_level {
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
    }

    fn flush(&self) {
        self.console.flush();
    }
}

/// Installs the global logger and returns a handle to the shared log
/// buffers for the UI to read from. Call once, before spawning any
/// threads that log. `ui_max_level` controls the UI panels only --
/// console output is controlled independently by the RUST_LOG
/// environment variable (standard env_logger behavior: nothing prints
/// if RUST_LOG isn't set at all).
pub fn init(ui_max_level: LevelFilter) -> &'static SharedLogs {
    let logs: &'static SharedLogs = Box::leak(Box::new(SharedLogs::new()));
    let console = env_logger::Builder::from_env(env_logger::Env::default()).build();
    let logger: &'static BufferLogger = Box::leak(Box::new(BufferLogger {
        logs,
        ui_max_level,
        console,
    }));
    log::set_logger(logger).expect("logger already initialized");
    // Trace globally -- deliberately permissive so records reach both
    // sinks above, which each apply their own (different) threshold
    // rather than one shared cutoff filtering records before either
    // sink sees them.
    log::set_max_level(LevelFilter::Trace);
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
