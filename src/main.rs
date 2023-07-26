/*
@author cnelson
Raspberry Pi Zero project


Project to read from a AHT20 Temperature sensor, 
convert to degrees C and relative humidity, 

*/

/* Crates & Libraries */
use std::error::Error;
use std::thread;
use std::time::Duration;

use rand::prelude::*;

use rppal::hal::Delay;
use rppal::gpio::Gpio;
use rppal::system::DeviceInfo;
// Use I2C
use rppal::i2c::I2c;
// Use UART (/dev/ttyS0)
use rppal::uart::{Parity, Uart};

// crate for serialize/deserialize of structs with #no_std support
use serde::{Deserialize, Serialize};

/* Local Imports*/
mod crc16;


/* Constants */
const GPIO_LED: u8 = 17; //RPI Zero W LED gpio 17

// Device Addr Select
const _AHT20_SLAVE_ADDR: u16 = 0x38 << 1 | 0x1;//0x38 with 1 bit for read == 0x71
const AHT20_SLAVE_ADDR_7BIT: u16 = 0x38; // RPI I2C requires 7 bit addr WITHOUT read/write bit
// Sensor Commands
const AHT20_INIT_CMD:   u8 = 0xBE;//
const AHT20_MEAS_CMD:   u8 = 0xAC;//
const _AHT20_RST_CMD:    u8 = 0xBA;//
// Masks
const _AHT20_BUSY_MSK:   u8 = 0x80;// Busy?
const AHT20_CAL_EN_MSK: u8 = 0x08;// Calibrated?
// Sync word
const SYNC_BYTE_UPPER: u8 = 0xFE;
const SYNC_BYTE_LOWER: u8 = 0xCA;
// Static packet length
const PACKET_LENGTH: u8 = 12;

/*

Sensor Packet - Minimal Implementation

Packet Structure
- sync_byte   u16 (Sync Word)    
- data_0      u16 (Temperature)
- data_1      u16 (Humidity)
- data_2      u16 (Time)
- data_3      u16 (Other)
- crc_16_byte u16 (CRC)

 */

#[repr(packed)]
#[derive(Serialize, Deserialize, Debug, Default)]
struct Packet {
    sync: u16,
    data_0: u16,
    data_1: u16,
    data_2: u16,
    data_3: u16,
    crc: u16,
}

impl Packet {
    /* Create a default impl for Packet struct. When this is called,
      it creates a new struct with default values for each data type*/
    fn new() -> Self {
        Default::default()
    }
}


fn fill_packet(pkt : &mut Packet, pkt_data_0 : u16, pkt_data_1: u16,  pkt_data_2: u16, pkt_data_3: u16){
    /* Packet: 
     - pass ref of mut Packet
     - modify what is mut Packet with fill data
     - return it (NOT a copy of it)  
     */
    
    // Fill the struct with sensor data and sync bytes
    pkt.sync = ((SYNC_BYTE_UPPER as u16) << 8) | SYNC_BYTE_LOWER as u16;
    pkt.data_0 = pkt_data_0;
    pkt.data_1 = pkt_data_1;
    pkt.data_2 = pkt_data_2;
    pkt.data_3 = pkt_data_3;


    // Create a buffer of bytes for CRC computation (TODO make this better)
    let mut buffer : [u8 ; (PACKET_LENGTH-2) as usize] = [
                                            pkt.sync.to_le_bytes()[0],pkt.sync.to_le_bytes()[1], 
                                            pkt_data_0.to_le_bytes()[0], pkt_data_0.to_le_bytes()[1],
                                            pkt_data_1.to_le_bytes()[0], pkt_data_1.to_le_bytes()[1],
                                            pkt_data_2.to_le_bytes()[0], pkt_data_2.to_le_bytes()[1],
                                            pkt_data_3.to_le_bytes()[0], pkt_data_3.to_le_bytes()[1]
                                            //Do not include default CRC in calc
                                            ];
                                            
    let new_crc : u16 = crc16::crc16_table(&mut buffer);
    //Finally, fill the packet crc
    pkt.crc = new_crc;

    //If we do not return explicitly, deconstructor will give back borrowed mut Packet
    //pkt;
}




fn aht20_init( i2c_bus: &mut I2c ) {

    /*
    Device requires at least 40ms since power on to start writing.
    This is here to ensure that is always true regardless of application
    */
    thread::sleep(Duration::from_millis(40));
    

    // Set the I2C slave address to the device we're communicating with.
    i2c_bus.set_slave_address(AHT20_SLAVE_ADDR_7BIT).unwrap();
    //     Ok() => {
    //         println!(" Slave Addr Set: {}", AHT20_SLAVE_ADDR_7BIT);
    //     },
    //     Err(msg)=>{
    //         println!("{}",msg);
    //         return Err(());
    //     }

    // }

    // Init the AHT20 temperature and humidity sensor.
    println!(" Init CMD: {}", AHT20_INIT_CMD);
    i2c_bus.write(&[AHT20_INIT_CMD]).unwrap();
    //     Ok(u)=>{
    //         println!("AHT20 write success: Bytes written={}",u);
    //     },
    //     Err(msg)=>{
    //         println!("{}",msg);
    //     }
    // };


    //Check CAL bit
    let mut read_buffer : [u8 ; 7] = [0x0,0x0,0x0,0x0,0x0,0x0,0x0];

    i2c_bus.write_read( &[0x71], &mut read_buffer);
    //Status byte is byte 0
    
    if ((read_buffer[0] & AHT20_CAL_EN_MSK) >> 3) != 0{
        //Calibrated
        println!( "cal {:?}", read_buffer[0]);
    }
    else{
        //Not calibrated
        println!( "uncal {:?}", read_buffer[0]);
    }
    // If we reach the end of the function, we can return OK
    //Ok(())
}

fn aht20_measure( i2c_bus: &mut I2c){//}, temp_buffer : & mut u16, hum_buffer : & mut u16, crc_buffer : & mut u8){
    //
    let mut read_buffer : [u8 ; 7] = [0x0,0x0,0x0,0x0,0x0,0x0,0x0];

    //Attempt read sensor
    i2c_bus.block_write(0x70,&[AHT20_MEAS_CMD,0x00 ,0x33] );
    //Wait for measurement to complete
    thread::sleep(Duration::from_millis(80));
    //Read bytes
    i2c_bus.block_read(0x71, &mut read_buffer);
    
    println!("sense : {:?} ", read_buffer);
    // Example output::
    //          [status, RH[0], RH[1], RH_t, TMP[0], TMP[1], CRC]
    // sense :  [156,    139,   174,   117,  246,    241,    88] 
    // sense :  [156,    139,   170,   69,   246,    207,    81] 


/*
sense : [156, 192, 51, 101, 170, 210, //129] 
        [9C, C0, 33, 65, AA, D2, //81]
 RH(%) = 75.07839
 T(C*) = 20.840836
crc_aht20 : 81
 crc_new : 4E
//computed result - 0x5C
 */

    let mut humidity : u32   = read_buffer[1].into();                          //20-bit raw humidity data
    humidity <<= 8;
    humidity  |= (read_buffer[2] as u32);
    humidity <<= 4;
    humidity  |= ((read_buffer[3] as u32) >> 4);
    //print!("raw {}  -> ", humidity);
    println!(" RH(%) = {}", 100.0 * ((humidity as f32) / 1048576.0) ); //(f32) 2^20 == 1048576.0

    let mut temperature : u32 = (read_buffer[3] & 0x0F).into();                //20-bit raw temperature data
    temperature <<= 8;
    temperature  |= (read_buffer[4] as u32);
    temperature <<= 8;
    temperature  |= (read_buffer[5] as u32);
    //print!("raw {}  -> ", temperature);
    println!(" T(C*) = {}",  (((temperature as f32) / 1048576.0)  * 200.0) - 50.0 ); //(f32) 2^20 == 1048576.0


    
    //Finally, verify with CRC8 bytes

    /* /
    "After receiving six bytes, the next byte is the CRC check
    data, the user can read it as needed, if the receiving end
    needs CRC check, then send it after receiving the sixth byte
    ACK response, otherwise NACK is sent out, CRC initial value
    is 0XFF, CRC8 check polynomial is:"

    CRC[7:0]=1+x^4 + x^5 + x^8
    b100110001 == 0x131 =? 8 bit = 0x31
    */

    let crc_aht20 : u8 = read_buffer[6]; 
    let mut crc_new : u8 = 0x00;//0xFF
    println!("crc_aht20 : {:X}" , crc_aht20);


    println!(" crc_new : {:X}" , crc_new);
}


fn main() -> Result<(), Box<dyn Error>> {
    println!("Hello Worlds!");
    println!("Blinking LED on {}.", DeviceInfo::new()?.model());

    let mut pin: rppal::gpio::OutputPin = Gpio::new()?.get(GPIO_LED)?.into_output();

    let mut rp_i2c: I2c = I2c::new()?;
    let mut uart: Uart = Uart::new(115_200, Parity::None, 8, 1)?; //Standard Baud
    let mut _delay : Delay = Delay::new(); //DelayMs

    aht20_init(&mut rp_i2c); // have to specify '&mut' for i2c_bus 

    //Uart configure
    uart.set_write_mode(false)?;

    loop { 
        //Blinky boi
        pin.set_high();
        println!("State is High: {}", pin.is_set_high());
        
        thread::sleep(Duration::from_millis(500));

        pin.set_low();
        println!("State is Low: {}", pin.is_set_low());

        thread::sleep(Duration::from_millis(500));

        let mut my_backpack : Packet = Packet::new();

        let temp: u16 = random::<u16>();
        let humd: u16 = random::<u16>();

        fill_packet(&mut my_backpack, temp, humd, 0xDE, 0xBE);

        /* Use JSON serializer  */
        // Convert the struct to a JSON string.
        //let serialized = serde_json::to_string(&my_backpack).unwrap();
        // Prints JSON serialized 
        //println!("serialized = {}", serialized);

        // Convert the JSON string back to a struct.
        //let deserialized: Packet = serde_json::from_str(&serialized).unwrap();
        // Prints Packet deserialized 
        //println!("deserialized = {:?}", deserialized);

        /* Use bytes serializer (bincode) */
        let encoded: Vec<u8> = bincode::serialize(&my_backpack).unwrap();
        //println!("encoded [{:?}]", encoded);
        
        // Deserialize a slice of the whole encoded vector (denoted by [start..end] )
        let decoded: Packet = bincode::deserialize(&encoded[..]).unwrap();
        //println!("decoded [{:?}]", decoded);

        //Send Data from random buffer
        let mut write_buffer: [u8; 2] = [ 0xab, rand::random::<u8>() ];
        let sent_bytes = uart.write(&mut write_buffer)?;
        //Send data from encoded raw bytes from sliced vector
        let write_vec_buffer: &[u8] = &encoded[..];
        let sent_vec_bytes : usize = uart.write( write_vec_buffer )?;
        //Total bytes sent
        //println!("Sent bytes : {} \n\n", sent_bytes + sent_vec_bytes);
        aht20_measure(&mut rp_i2c);

    }


}

