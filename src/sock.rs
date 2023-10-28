use std::net::TcpStream;
use std::{net::TcpListener, thread::spawn};
use tungstenite::stream::MaybeTlsStream;
use tungstenite::{accept, WebSocket};
use tungstenite::{connect, Message};
use url::Url;


use tungstenite::{
    accept_hdr,
    handshake::server::{Request, Response},
};


// pub struct WebSocks{
//     status :u8,
//     web_server_sock : WebSocket<TcpStream>,
//     web_client_sock : WebSocket<MaybeTlsStream<TcpStream>>
// }


pub fn server_register() {//-> WebSocket<TcpStream>
    // env_logger::init();
    let server = TcpListener::bind("127.0.0.1:3012").unwrap();
    for stream in server.incoming() {
        spawn(move || {
            let callback = |req: &Request, mut response: Response| {
                println!("Received a new ws handshake");
                println!("The request's path is: {}", req.uri().path());
                println!("The request's headers are:");
                for (ref header, _value) in req.headers() {
                    println!("* {}", header);
                }

                // Let's add an additional header to our response to the client.
                let headers = response.headers_mut();
                headers.append("MyCustomHeader", ":)".parse().unwrap());
                headers.append("SOME_TUNGSTENITE_HEADER", "header_value".parse().unwrap());

                Ok(response)
            };
            let mut websocket = accept_hdr(stream.unwrap(), callback).unwrap();

            loop {
                let msg = websocket.read().unwrap();
                if msg.is_binary() || msg.is_text() {
                    websocket.send(msg).unwrap();
                }
            }
        });
    }
}

pub fn client_register(address : &str) -> WebSocket<MaybeTlsStream<TcpStream>>{
    // env_logger::init();
    // Default Addr : "ws://localhost:3012/socket"
    let (mut socket, response) =
        connect(Url::parse(address).unwrap()).expect("Can't connect");

    println!("Connected to the server");
    println!("Response HTTP code: {}", response.status());
    println!("Response contains the following headers:");
    for (ref header, _value) in response.headers() {
        println!("* {}", header);
    }

    return socket;
}

pub fn client_message(socket : &mut WebSocket<MaybeTlsStream<TcpStream>>, payload : String){

    socket.send(Message::Text(payload.into())).unwrap();

    let msg =  socket.read().expect("Error reading message...");
    println!("Received: {}", msg);
}
