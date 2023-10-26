/*
@author cnelson
Raspberry Pi Zero project


Project to read sensors from various sources 
and send data to a server

Device 1: Aht20 Temperature sensor
    - Temperature C
    - Humidity RH
Device 2: Pi Pico 
    - Onboard Temperature C
    - Max31856 with K-Type Thermistor Temperature C 

*/

/* Crates & Libraries */
use std::error::Error;
use std::{thread, time};
use std::io::{self, Write};

use serialport::{available_ports, SerialPort};

use rand::prelude::*;

// use rppal::hal::Delay;
use rppal::gpio::Gpio;
use rppal::system::DeviceInfo;
// Use I2C
use rppal::i2c::I2c;
// Use UART (/dev/ttyS0)
use rppal::uart::{Parity, Uart};

// crate for serialize/deserialize of structs with #no_std support
use serde::{Deserialize, Serialize};

/* Local Imports*/
mod crc;

mod sock;


/* Constants */
const GPIO_LED: u8 = 17; //RPI Zero W LED gpio 17
const GPIO_DATA_REQ: u8 = 27; //gpio 27

// Device Addr Select
const AHT20_SLAVE_ADDR_7BIT: u16 = 0x38; // RPI I2C requires 7 bit addr WITHOUT read/write bit
// Sensor Commands
const AHT20_INIT_CMD:   u8 = 0xBE;//
const AHT20_MEAS_CMD:   u8 = 0xAC;//
const AHT20_READ_CMD:   u8 =  0x38 << 1 | 0x1;//0x38 with 1 bit for read  == 0x71
const AHT20_WRITE_CMD:  u8 =  0x38 << 1 | 0x0;//0x38 with 1 bit for write == 0x70
const _AHT20_RST_CMD:   u8 = 0xBA;//
// Masks
const _AHT20_BUSY_MSK:   u8 = 0x80;// Busy?
const AHT20_CAL_EN_MSK: u8 = 0x08;// Calibrated?
// Sync word
const SYNC_BYTE_UPPER: u8 = 0xFE;
const SYNC_BYTE_LOWER: u8 = 0xCA;
// Static packet length
const PACKET_LENGTH: u8 = 20;


//
#[repr(packed)]
#[derive(Serialize, Deserialize, Debug, Default)]
struct Aht20Data{
    //status
    status: u8,
    //crc8
    checksum: u8,
    //Raw : u32
    u32_temperature: u32,
    u32_humidity : u32,
    //Converted : f32
    f32_temperature : f32,
    f32_humidty: f32,
}

impl Aht20Data {
    /*
    Default Data packet for AHT20
    */
    fn new() -> Self {
        Default::default()
    }
    fn collect(&mut self, buffer : &[u8] ) -> Result<(), ()>{

        //Check buffer is correct length
        if buffer.len() == 7{

            let checked_crc = crc::crc8_aht20(&buffer);

            //If crc check passes, proceed to fill the rest of the data
            if(checked_crc == buffer[6]){
                self.checksum = checked_crc;
                self.status = buffer[0];

                let mut humidity : u32   = buffer[1].into(); //20-bit raw humidity data -> u32
                humidity <<= 8;
                humidity  |= buffer[2] as u32;
                humidity <<= 4;
                humidity  |= (buffer[3] as u32) >> 4;
                self.u32_humidity = humidity;
                self.f32_humidty = 100.0 * ((humidity as f32) / 1048576.0);//(f32) 2^20 == 1048576.0
                // println!("Raw Humidity = {},  RH(%) = {}", humidity,  100.0 * ((humidity as f32) / 1048576.0)); 
            
                let mut temperature : u32 = (buffer[3] & 0x0F).into(); //20-bit raw temperature data -> u32
                temperature <<= 8;
                temperature  |= buffer[4] as u32;
                temperature <<= 8;
                temperature  |= buffer[5] as u32;
                self.u32_temperature = temperature;
                self.f32_temperature =  (((temperature as f32) / 1048576.0)  * 200.0) - 50.0;//(f32) 2^20 == 1048576.0
                //println!("Raw Temperature = {}, T(C*) = {}", temperature, (((temperature as f32) / 1048576.0)  * 200.0) - 50.0); 
                
                //Successful data collection, return Ok
                Ok(())
            }
            //Otherwise, inform of failure
            else{
                println!("CRC Failed! crc::crc8_aht20 == {} , aht20_crc == {}", checked_crc, buffer[6]);
                Err(())
            }
        }
        else{
            println!("Expected buffer.len() == 7, instead buffer.len() == {}", buffer.len());
            Err(())
        }
    

    }

    
}

/*
Sensor Packet - Minimal Implementation

Packet Structure
- sync_bytes   u16 (Sync Word)    
- data_0      u32 (Aht20 Temperature)
- data_1      u32 (Ktype Thermistor Temp)
- data_2      u32 (Humidity)
- data_3      u16 (Time)
- data_4      u16 (Pico Onboard temp)
- crc_16_byte u16 (CRC)

 */

#[repr(packed)]
#[derive(Serialize, Deserialize, Debug, Default)]
struct Packet {
    /* Generic data packet containing sync bytes, u32 & u16 data, and a crc */
    sync: u16,
    data_0: u32,
    data_1: u32,
    data_2: u32,
    data_3: u16,
    data_4: u16,
    crc: u16,
}

impl Packet {
    /* Create a default impl for Packet struct. When this is called,
      it creates a new struct with default values for each data type*/
    fn new() -> Self {
        Default::default()
    }
}

fn fill_packet(pkt : &mut Packet, pkt_data_0 : u32, pkt_data_1: u32, pkt_data_2: u32,  pkt_data_3: u16, pkt_data_4: u16){
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
    pkt.data_4 = pkt_data_4;


    // Create a buffer of bytes for CRC computation (TODO make this better)
    let mut buffer : [u8 ; (PACKET_LENGTH-2) as usize] = [
                                            //SYNC
                                            pkt.sync.to_le_bytes()[0],pkt.sync.to_le_bytes()[1], 
                                            //Aht20 Temp
                                            pkt_data_0.to_le_bytes()[0], pkt_data_0.to_le_bytes()[1],
                                            pkt_data_0.to_le_bytes()[2], pkt_data_0.to_le_bytes()[3],
                                            //Ktype Temp
                                            pkt_data_1.to_le_bytes()[0], pkt_data_1.to_le_bytes()[1],
                                            pkt_data_1.to_le_bytes()[2], pkt_data_1.to_le_bytes()[3],
                                            //Hum
                                            pkt_data_2.to_le_bytes()[0], pkt_data_2.to_le_bytes()[1],
                                            pkt_data_2.to_le_bytes()[2], pkt_data_2.to_le_bytes()[3],
                                            //Time TODO
                                            pkt_data_3.to_le_bytes()[0], pkt_data_3.to_le_bytes()[1],
                                            //Pico onboard temp
                                            pkt_data_4.to_le_bytes()[0], pkt_data_4.to_le_bytes()[1]
                                            //Do not include default CRC in calc
                                            ];
                                            
    let new_crc : u16 = crc::crc16_table(&mut buffer);
    //Finally, fill the packet crc
    pkt.crc = new_crc;

    //If we do not return explicitly, deconstructor will give back borrowed mut Packet
    //pkt;
}




fn aht20_init( i2c_bus: &mut I2c ) -> Result<(), ()>{
    /*
    Device requires at least 40ms since power on to start writing.
    This is here to ensure that is always true regardless of application
    */
    thread::sleep(time::Duration::from_millis(40));
    
    // Set the I2C slave address to the device we're communicating with (MUST BE u16!)
    match i2c_bus.set_slave_address(AHT20_SLAVE_ADDR_7BIT){
        Ok(()) => {
            println!("Slave Addr Set: {}", AHT20_SLAVE_ADDR_7BIT);
        },
        Err(msg)=>{
            println!("Unable to set Slave to: {}... {}", AHT20_SLAVE_ADDR_7BIT, msg);
            return Err(());
        }
    };

    // Init the AHT20 temperature and humidity sensor.
    println!("Init CMD: {}", AHT20_INIT_CMD);
    match i2c_bus.write(&[AHT20_INIT_CMD]){
        Ok(u)=>{
            println!("AHT20 write success: Bytes written = {}",u);
        },
        Err(msg)=>{
            println!("Unable to init AHT20 {}",msg);
            return Err(());
        }
    };

    //Check CAL bit
    let mut read_buffer : [u8 ; 7] = [0x0,0x0,0x0,0x0,0x0,0x0,0x0];

    match i2c_bus.write_read( &[AHT20_READ_CMD], &mut read_buffer){
        Ok(())=>{
            println!("AHT20 i2c.write_read success");
        },
        Err(msg)=>{
            println!("AHT20 i2c.write_read failed! {}",msg);
            return Err(());
        }
    };
    //Status byte is byte 0
    if ((read_buffer[0] & AHT20_CAL_EN_MSK) >> 3) != 0{
        //Calibrated
        println!( "Calibrated {:?}", read_buffer[0]);
    }
    else{
        //Not calibrated
        println!( "Not Calibrated {:?}", read_buffer[0]);
    }
    // If we reach the end of the function, we can return OK
    Ok(())
}

fn aht20_measure( i2c_bus: &mut I2c) -> Result<Aht20Data, ()>{
    //Default buffer
    let mut read_buffer : [u8 ; 7] = [0x0,0x0,0x0,0x0,0x0,0x0,0x0];
    //Attempt read sensor
    match i2c_bus.block_write(AHT20_WRITE_CMD,&[AHT20_MEAS_CMD,0x00 ,0x33]){
        Ok(())=>{
            //pass
        },
        Err(msg)=>{
            println!("Error in aht20_measure -> i2c.block_write(...) : {}", msg);
            return Err(());
        }
    };
    //Wait for measurement to complete
    thread::sleep(time::Duration::from_millis(80));
    //Read bytes
    match i2c_bus.block_read(AHT20_READ_CMD, &mut read_buffer){
        Ok(())=>{
            //pass
        },
        Err(msg )=>{
            println!("Error in aht20_measure -> i2c.block_read(...) : {}", msg);
            return Err(());
        }
    };
    //Put into Aht20Data struct
    let mut aht20_data : Aht20Data = Aht20Data::new();
    match Aht20Data::collect(&mut aht20_data, &read_buffer){
        Ok(()) => {
            //Return struct of our AHT20 data
            return Ok(aht20_data);
        }
        Err(()) => {
            println!("Error in aht20_measure -> Aht20Data::collect(...) ");
            return Err(());
        }
    };
}


fn main() -> Result<(), Box<dyn Error>> {
    /** INITIALIZATION **/
    println!("Hello Worlds!");
    println!("Blinking LED on {}.", DeviceInfo::new()?.model());
    println!{"{:?}", time::Instant::now()};

    let mut pin: rppal::gpio::OutputPin = Gpio::new()?.get(GPIO_LED)?.into_output();
    let mut data_req_pin : rppal::gpio::OutputPin = Gpio::new()?.get(GPIO_DATA_REQ)?.into_output();
    data_req_pin.set_high();

    let mut rp_i2c: I2c = I2c::new()?;
    let mut uart: Uart = Uart::new(115_200, Parity::None, 8, 1)?; //Standard Baud

    // Socket Client instantiate
    // sock::client_main();


    /* I2c device init */
    match aht20_init(&mut rp_i2c){
        Ok(()) =>{
            println!("Init success!");
        }
        Err(()) => {
            panic!("Unable to initialize AHT20 sensor!")
        }
    }
    /* Serial wire UART configure */
    uart.set_write_mode(false)?;

    /* Pi Pico serial port */
    let port_name = "/dev/ttyACM0";
    let baud_rate = 115200;
    let mut port = serialport::new(port_name, baud_rate)
        .timeout(time::Duration::from_millis(250)).open()?;

    /** LOOP FOREVER **/
    loop { 

        /* Send Data Request - active low */
        data_req_pin.set_low();

        /* Current Time */
        println!{"{:?}", time::Instant::now()};

        /*
        Packet Len = 13
        Composition = [ SYNC_BYTE_LOWER, SYNC_BYTE_UPPER, 
                    COUNTER_BYTE_0, COUNTER_BYTE_1, COUNTER_BYTE_2, COUNTER_BYTE_3, 
                    TEMP_ONBOARD_BYTE_LOWER, TEMP_ONBOARD_BYTE_UPPER, 
                    THERMO_BYTE_0, THERMO_BYTE_1, THERMO_BYTE_2, THERMO_BYTE_3, 
                    END_OF_LINE_0A
                    ]
        */

        /* Serial Buffer for read response */
        let mut serial_buf: Vec<u8> = vec![0; 14];
        //println!("Receiving data on {} at {} baud:", &port_name, &baud_rate);
        match port.read(serial_buf.as_mut_slice()) {
            /*
            If we get a successful read from the pico, 
            then continue the program, otherwise throw 
            an Err and retry!
             */
            Ok(t) => {
                //Write serial port bytes to buffer
                io::stdout().write_all(&serial_buf[..t]).unwrap();
                /* Debug print data */
                // print!("\n[");
                // for byte in &serial_buf{
                //     print!("{:X} ",byte);
                // }
                // println!("]");
                /* Check Framing */
                if(serial_buf[0] != 0xCA || serial_buf[1] != 0xFE || serial_buf[13] != 0xA){
                    println!("Framing error : moving on to next packet...");
                    continue;
                }
                /* Read the AHT20 */
                match aht20_measure(&mut rp_i2c){
                    Ok(my_aht20) =>{
                        //Packet
                        let mut my_backpack : Packet = Packet::new();

                        let mut pico_onboard_temp: u16 = (serial_buf[6] & 0xFF).into(); //
                        pico_onboard_temp <<= 8;
                        pico_onboard_temp  |= serial_buf[7] as u16;

                        let mut ktype : u32 = (serial_buf[8] & 0xFF).into(); //
                        ktype <<= 8;
                        ktype  |= serial_buf[9] as u32;
                        ktype <<= 8;
                        ktype  |= serial_buf[10] as u32;
                        ktype <<= 8;
                        ktype  |= serial_buf[11] as u32;

                        // TODO time stamp
                        let data_4: u16 = random::<u16>();

                        // TODO send over socket
                        fill_packet(&mut my_backpack, my_aht20.u32_temperature,  ktype,my_aht20.u32_humidity,  data_4, pico_onboard_temp,);
                        /* Use bytes serializer (bincode) */
                        let encoded: Vec<u8> = bincode::serialize(&my_backpack).unwrap();
                        /* Example: serialized:
                             [202, 254, 71, 120, 5, 0, 196, 192, 11, 0, 0, 0, 0, 29, 231, 57, 3, 3, 203, 154] 
                             [202, 254] [71, 120, 5, 0] [196, 192, 11, 0] [0, 0, 0, 29] [231, 57] [3, 3] [203, 154] 
                        */
                        println!("serialized: {:?}",encoded);

                    },
                    Err(()) =>{
                        //Unable to read sensor, continue TODO handle errror
                        continue;
                    }
                };
                //Blinky boi to let us know data has been recv
                pin.toggle();
                println!("State is : {}", pin.is_set_high());
            //End Ok()
            },
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
            Err(e) => eprintln!("{:?}", e),
        }
        /* Reset data request pin high */
        data_req_pin.set_high();

        /* Sleep */
        thread::sleep(time::Duration::from_millis(2000));
    
    }


}

