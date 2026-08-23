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
use std::time::SystemTime;

pub const MAX_MESSAGES: usize = 100;

#[derive(Clone)]
pub struct LogEntry {
    pub level: Level,
    pub text: String,
    /// Wall-clock time the entry was logged -- SystemTime (not Instant)
    /// specifically so it can be rendered as an HH:MM:SS.mmm timestamp
    /// column in the UI.
    pub at: SystemTime,
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
                at: SystemTime::now(),
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

/// HH:MM:SS.mmm (UTC, same as worker.rs's format_time_hms, but with
/// millisecond precision since log lines can arrive faster than once a
/// second).
fn format_timestamp(at: SystemTime) -> String {
    let dur = at.duration_since(std::time::UNIX_EPOCH).unwrap_or_default();
    let secs = dur.as_secs();
    let millis = dur.subsec_millis();
    let (h, m, s) = ((secs / 3600) % 24, (secs / 60) % 60, secs % 60);
    format!("{h:02}:{m:02}:{s:02}.{millis:03}")
}

/// Fixed widths for the timestamp/severity columns, in points -- wide
/// enough for "23:59:59.999" and the full word "Severity" (the header,
/// which is the widest thing in that column) respectively, plus the
/// Frame margins in header_cell_ui/cell_ui below. Both are still
/// user-resizable (see Column::resizable below).
const TIMESTAMP_COL_WIDTH: f32 = 104.0;
const SEVERITY_COL_WIDTH: f32 = 92.0;

/// egui_table delegate for one port's log. Holds a borrow of the
/// messages for the duration of one `show_panel` call.
struct LogTableDelegate<'a> {
    entries: &'a VecDeque<LogEntry>,
}

impl egui_table::TableDelegate for LogTableDelegate<'_> {
    fn header_cell_ui(&mut self, ui: &mut eframe::egui::Ui, cell: &egui_table::HeaderCellInfo) {
        // egui_table adds no cell margins itself (see crate docs) -- a
        // Frame with inner_margin is what stops adjacent header/cell
        // text from touching the next column.
        eframe::egui::Frame::new()
            .inner_margin(eframe::egui::Margin::symmetric(6, 2))
            .show(ui, |ui| {
                let title = match cell.col_range.start {
                    0 => "Time",
                    1 => "Severity",
                    _ => "Message",
                };
                ui.strong(title);
            });
    }

    fn cell_ui(&mut self, ui: &mut eframe::egui::Ui, cell: &egui_table::CellInfo) {
        use eframe::egui::{Frame, Margin, RichText};

        let Some(entry) = self.entries.get(cell.row_nr as usize) else {
            return;
        };
        let color = level_color(ui, entry.level);
        Frame::new().inner_margin(Margin::symmetric(6, 2)).show(ui, |ui| {
            match cell.col_nr {
                0 => {
                    ui.label(RichText::new(format_timestamp(entry.at)).monospace().weak());
                }
                1 => {
                    ui.label(RichText::new(entry.level.to_string()).monospace().color(color));
                }
                _ => {
                    ui.label(RichText::new(&entry.text).color(color));
                }
            }
        });
    }

    fn default_row_height(&self) -> f32 {
        20.0
    }
}

/// Renders one port's log as a table: Time | Severity | Message, with a
/// user-draggable divider between every column. `title` must be unique
/// across panels shown in the same frame (used as the table's id salt).
pub fn show_panel(ui: &mut eframe::egui::Ui, title: &str, port_log: &PortLog) {
    use eframe::egui::Rangef;

    ui.vertical(|ui| {
        ui.horizontal(|ui| {
            ui.strong(title);
            ui.label(format!("{} message(s)", port_log.messages.len()));
        });

        let mut delegate = LogTableDelegate {
            entries: &port_log.messages,
        };

        egui_table::Table::new()
            .id_salt(title)
            .num_rows(port_log.messages.len() as u64)
            .columns(vec![
                // Kept narrow-ranged so AutoSizeMode::OnParentResize below
                // has almost no room to grow these -- extra/deficit width
                // goes to the Message column instead. Still resizable by
                // hand within that narrow band.
                egui_table::Column::new(TIMESTAMP_COL_WIDTH)
                    .range(Rangef::new(TIMESTAMP_COL_WIDTH - 4.0, TIMESTAMP_COL_WIDTH + 24.0))
                    .resizable(true),
                egui_table::Column::new(SEVERITY_COL_WIDTH)
                    .range(Rangef::new(SEVERITY_COL_WIDTH - 4.0, SEVERITY_COL_WIDTH + 16.0))
                    .resizable(true),
                // Not resizable, and given a wide-open range (the
                // Column default, 4.0..INFINITY) -- see auto_size_mode
                // below for how this actually gets filled.
                egui_table::Column::new(160.0),
            ])
            .headers([egui_table::HeaderRow::new(20.0)])
            // AutoSizeMode::Always, not OnParentResize: OnParentResize
            // only recomputes when the table's parent width *changes*
            // between frames (state.parent_width != Some(parent_width)).
            // Only *resizable* columns get their width written into that
            // persisted state, though -- this Message column is
            // deliberately non-resizable, so its width is never
            // persisted and resets to the 160.0 seed above at the top of
            // every single frame. With OnParentResize, once the parent
            // width stops changing (i.e. after the first frame or two),
            // auto_size stops re-running and the column is stuck back at
            // that 160.0 seed forever. Always recomputes it every frame
            // unconditionally, which is what a non-persisted column
            // actually needs.
            .auto_size_mode(egui_table::AutoSizeMode::Always)
            .stick_to_bottom(true)
            .show(ui, &mut delegate);
    });
}
