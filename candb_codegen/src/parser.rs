use std::collections::HashMap;

use convert_case::{Case, Casing};


#[derive(Default, Debug, Clone)]
pub struct Frame {
    pub name: String,
    pub id: u16,
    pub signals: Vec<SignalBasic>,
}

#[derive(Default)]
pub struct CanDbParser {
    pub curr_ecu: String,
    pub ecus: HashMap<String, Vec<Frame>>,
}

#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub enum DataType {
    Bool,
    Int {
        offset: f32,
        multi: f32
    },
    Enum(Vec<(u32, String, String)>),
    IsoTp,
}

#[derive(Debug, Clone)]
pub struct SignalBasic {
    pub name: String,
    pub desc: String,
    pub offset: u32,
    pub len: u32,
    pub dt: DataType
}

impl CanDbParser {
    /// Name, Descr
    pub fn basic_data(line: &str) -> SignalBasic {
        let mut p = line.split(", ");
        let name = p.next().unwrap().trim().split(" ").last().unwrap().to_case(Case::UpperCamel);
        let offset = p.next().unwrap().trim().split(": ").last().unwrap().parse::<u32>().unwrap();
        let len = p.next().unwrap().trim().split(": ").last().unwrap().parse::<u32>().unwrap();
        let dt_str = line.split("DATA TYPE ").last().unwrap().trim();
        let desc = line.split("DESC: ").last().unwrap().split(", DATA TYPE").next().unwrap();
        let dt = if dt_str.starts_with("ENUM") {
            DataType::Enum(vec![])
        } else if dt_str.starts_with("BOOL") {
            DataType::Bool
        } else if dt_str.starts_with("NUMBER") {
            let multi: f32 = dt_str.split("_MULTIPLIER_: ").last().unwrap().split(",").next().unwrap().parse().unwrap();
            let offset: f32 = dt_str.split("_OFFSET_: ").last().unwrap().split(")").next().unwrap().parse().unwrap();
            DataType::Int { offset, multi }
        } else if dt_str.starts_with("ISO_TP") {
            DataType::IsoTp
        } else {
            panic!("Invalid data type {dt_str}");
        };
        SignalBasic { name: name.to_string(), desc: desc.to_string(), offset, len, dt }
    }


    pub fn parse_file(&mut self, contents: String) {
        for line in contents.lines() {
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with("#"){} // Comment of newline
            else if let Some(ecu_name) = trimmed.strip_prefix("ECU ") {
                self.new_ecu(ecu_name);
            } else if trimmed.starts_with("FRAME ") {
                let mut parts = trimmed.split(" ");
                let _ = parts.next(); // FRAME
                let frame_name = parts.next().unwrap().trim();
                let frame_id = parts.next().unwrap().trim_matches(['(', ')']);
                let frame_id_int = u16::from_str_radix(&frame_id[2..], 16).unwrap();
                self.new_frame(frame_name, frame_id_int);
            } else if trimmed.starts_with("SIGNAL ") {
                let data = Self::basic_data(trimmed);
                if let Some(frame) = self.get_current_working_frame() {
                    frame.signals.push(data);
                }
            } else if trimmed.starts_with("ENUM ") {
                let name = trimmed.replace("ENUM ", "").split(",").next().unwrap().to_string();
                let raw = trimmed.split("RAW: ").last().unwrap().split(",").next().unwrap().parse::<u32>().unwrap();
                let desc = trimmed.split("DESC: ").last().unwrap();
                if let Some(frame) = self.get_current_working_frame() {
                    if let DataType::Enum(enum_tab) = &mut frame.signals.last_mut().unwrap().dt {

                        let mut n = name.to_string().to_case(Case::UpperCamel);
                        if n.is_empty() {
                            // Sometimes happens for enum variants `_`
                            n = "Underscore".to_string()
                        }
                        enum_tab.push((raw, n, desc.to_string()));
                    } else {
                        panic!("Adding enum to non enum signal");
                    }
                }
            } else {
                panic!("Invalid entry: {line}")
            }
        }
    }

    pub fn new_ecu(&mut self, name: &str) {
        self.curr_ecu = name.to_string();
        if self.ecus.contains_key(name) {
            panic!("ECU {name} already exists in DB!");
        }
        self.ecus.insert(name.to_string(), vec![]);
    }

    pub fn get_current_working_frame(&mut self) -> Option<&mut Frame> {
        let ecu = self.ecus.get_mut(&self.curr_ecu)?;
        ecu.last_mut()
    }

    pub fn new_frame(&mut self, name: &str, id: u16) {
        if let Some(frame_list) = self.ecus.get_mut(&self.curr_ecu) {
            frame_list.push(Frame { 
                name: name.to_string(), 
                id, 
                signals: vec![],
            });
        } else {
            panic!("Trying to add frame {name} to no valid ECU");
        }
    }
}

