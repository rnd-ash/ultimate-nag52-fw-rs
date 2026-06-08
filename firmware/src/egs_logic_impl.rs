

#[derive(Default)]
pub struct EgsDtcStorage;

impl egs_logic::StorageBacking for EgsDtcStorage {
    async fn init(&mut self, mode: &mut egs_logic::errors::DeviceMode) {
        
    }
}

impl egs_logic::GearboxDtcStorage for EgsDtcStorage {

}

#[derive(Default)]
pub struct EgsAdpStorage;

impl egs_logic::StorageBacking for EgsAdpStorage {
    async fn init(&mut self, mode: &mut egs_logic::errors::DeviceMode) {
        
    }
}

impl egs_logic::GearboxAdaptStorage for EgsAdpStorage {

}

#[derive(Default)]
pub struct EgsCalStorage;

impl egs_logic::StorageBacking for EgsCalStorage {
    async fn init(&mut self, mode: &mut egs_logic::errors::DeviceMode) {
        
    }
}

impl egs_logic::GearboxCalibStorage for EgsCalStorage {

}

#[derive(Default)]
pub struct EgsMapStorage;

impl egs_logic::StorageBacking for EgsMapStorage {
    async fn init(&mut self, mode: &mut egs_logic::errors::DeviceMode) {
        
    }
}

impl egs_logic::GearboxMapStorage for EgsMapStorage {

}


pub type V2Gearbox = egs_logic::Gearbox<
    EgsDtcStorage,
    EgsCalStorage,
    EgsMapStorage,
    EgsAdpStorage
>;