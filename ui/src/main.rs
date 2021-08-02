use std::env;
use std::fs::File;

fn main() -> std::io::Result<()> {
    let args: Vec<String> = env::args().collect();
    let filename = &args[1];
    let mut file = File::open(filename)?;
    let loader = system::get_file_loader(filename, &mut file).unwrap();
    let _ = loader.load(&mut file);
    Ok(())
}
