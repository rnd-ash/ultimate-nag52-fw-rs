use std::{fs, path::PathBuf};


mod code_gen;
mod parser;
use clap::Parser;

use crate::code_gen::CodeGenerator;

#[derive(Debug, clap::Parser)]
pub struct ParserSettings {
    layer_name: String,
    in_file: PathBuf,
    out_dir: PathBuf
}

fn main() {
    let settings = ParserSettings::parse();

    if settings.out_dir.is_file() {
        panic!("Out dir cannot be a file, must be a folder");
    }
    let in_file = fs::read(settings.in_file).unwrap();

    let s = String::from_utf8(in_file).unwrap();

    let mut candb_parser = parser::CanDbParser::default();
    candb_parser.parse_file(s);
    let code_generator = CodeGenerator::new(candb_parser.ecus, settings.out_dir);
    code_generator.code_gen(&settings.layer_name).unwrap();
}