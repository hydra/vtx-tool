use log::{Level, LevelFilter, Log, Metadata, Record};
use std::collections::VecDeque;
use std::sync::Mutex;
use std::time::SystemTime;

pub const MAX_MESSAGES: usize = 100;

#[derive(Clone)]
pub struct LogEntry {
    pub level: Level,
    pub text: String,
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
    ui_max_level: LevelFilter,
    console: env_logger::Logger,
}

impl Log for BufferLogger {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        if self.console.enabled(record.metadata()) {
            self.console.log(record);
        }

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

pub fn init(ui_max_level: LevelFilter) -> &'static SharedLogs {
    let logs: &'static SharedLogs = Box::leak(Box::new(SharedLogs::new()));
    let console = env_logger::Builder::from_env(env_logger::Env::default()).build();
    let logger: &'static BufferLogger = Box::leak(Box::new(BufferLogger {
        logs,
        ui_max_level,
        console,
    }));
    log::set_logger(logger).expect("logger already initialized");
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

fn format_timestamp(at: SystemTime) -> String {
    let dur = at.duration_since(std::time::UNIX_EPOCH).unwrap_or_default();
    let secs = dur.as_secs();
    let millis = dur.subsec_millis();
    let (h, m, s) = ((secs / 3600) % 24, (secs / 60) % 60, secs % 60);
    format!("{h:02}:{m:02}:{s:02}.{millis:03}")
}

const TIMESTAMP_COL_WIDTH: f32 = 104.0;
const SEVERITY_COL_WIDTH: f32 = 92.0;

struct LogTableDelegate<'a> {
    entries: &'a VecDeque<LogEntry>,
}

impl egui_table::TableDelegate for LogTableDelegate<'_> {
    fn header_cell_ui(&mut self, ui: &mut eframe::egui::Ui, cell: &egui_table::HeaderCellInfo) {
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
        Frame::new()
            .inner_margin(Margin::symmetric(6, 2))
            .show(ui, |ui| match cell.col_nr {
                0 => {
                    ui.label(RichText::new(format_timestamp(entry.at)).monospace().weak());
                }
                1 => {
                    ui.label(
                        RichText::new(entry.level.to_string())
                            .monospace()
                            .color(color),
                    );
                }
                _ => {
                    ui.label(RichText::new(&entry.text).color(color));
                }
            });
    }

    fn default_row_height(&self) -> f32 {
        20.0
    }
}

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
                egui_table::Column::new(TIMESTAMP_COL_WIDTH)
                    .range(Rangef::new(
                        TIMESTAMP_COL_WIDTH - 4.0,
                        TIMESTAMP_COL_WIDTH + 24.0,
                    ))
                    .resizable(true),
                egui_table::Column::new(SEVERITY_COL_WIDTH)
                    .range(Rangef::new(
                        SEVERITY_COL_WIDTH - 4.0,
                        SEVERITY_COL_WIDTH + 16.0,
                    ))
                    .resizable(true),
                egui_table::Column::new(160.0),
            ])
            .headers([egui_table::HeaderRow::new(20.0)])
            .auto_size_mode(egui_table::AutoSizeMode::Always)
            .stick_to_bottom(true)
            .show(ui, &mut delegate);
    });
}
