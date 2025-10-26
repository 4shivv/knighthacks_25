//
//  LocalWebServer.swift
//  LidarCapture
//
//  Created by nicky on 10/25/25.
//

import Foundation
import Network

class LocalWebServer {
    private var listener: NWListener?
    private let queue = DispatchQueue(label: "LocalWebServerQueue")
    private var latestFrameData: Data?
    
    func start(port: UInt16 = 8080) {
        do {
            listener = try NWListener(using: .tcp, on: NWEndpoint.Port(rawValue: port)!)
        } catch {
            print("❌ Failed to start listener: \(error)")
            return
        }
        
        listener?.newConnectionHandler = { [weak self] connection in
            connection.start(queue: self?.queue ?? .main)
            self?.handleConnection(connection)
        }
        
        listener?.start(queue: queue)
        print("🌐 Web server running on port \(port)")
    }
    
    func stop() {
        listener?.cancel()
        listener = nil
        print("🛑 Web server stopped")
    }
    
    func updateFrame(_ imageData: Data) {
        latestFrameData = imageData
    }
    
    private func handleConnection(_ connection: NWConnection) {
        connection.receive(minimumIncompleteLength: 1, maximumLength: 4096) { [weak self] data, _, _, _ in
            guard let self = self, let data = data, let request = String(data: data, encoding: .utf8) else {
                connection.cancel()
                return
            }
            
            if request.contains("GET /capture") {
                self.sendFrameResponse(connection)
            } else {
                self.sendTextResponse(connection, "Local LIDAR API\nUse /capture to get the current frame.")
            }
        }
    }
    
    private func sendFrameResponse(_ connection: NWConnection) {
        guard let frameData = latestFrameData else {
            sendTextResponse(connection, "❌ No frame available")
            return
        }
        
        let header = """
        HTTP/1.1 200 OK\r
        Content-Type: image/jpeg\r
        Content-Length: \(frameData.count)\r
        \r
        """
        var response = Data(header.utf8)
        response.append(frameData)
        
        connection.send(content: response, completion: .contentProcessed { error in
            if let error = error {
                print("❌ Send error: \(error)")
            }
            connection.cancel()
        })
    }
    
    private func sendTextResponse(_ connection: NWConnection, _ text: String) {
        let body = Data(text.utf8)
        let header = """
        HTTP/1.1 200 OK\r
        Content-Type: text/plain\r
        Content-Length: \(body.count)\r
        \r
        """
        var response = Data(header.utf8)
        response.append(body)
        
        connection.send(content: response, completion: .contentProcessed { error in
            if let error = error {
                print("❌ Send error: \(error)")
            }
            connection.cancel()
        })
    }
}
