# Build CLI Flasher app

[working-directory: 'flasher']
build_cli_app:
    cargo build --release

# Flash and run firmware
[working-directory: 'firmware']
run_fw *args:
    just build_cli_app
    cargo build --release
    ./../flasher/target/release/flasher {{args}} flash --application ../target/thumbv7em-none-eabihf/release/firmware -l