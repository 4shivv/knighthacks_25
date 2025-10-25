//
//  NetworkManager.swift
//  LidarCapture
//
//  Created by nicky on 10/25/25.
//
import Foundation
import Combine

class NetworkManager: ObservableObject {
    @Published var isConnected = false
    @Published var pointCloudDensity = 5
    
    private var serverURL: String = "http://localhost:5000"
    private var cancellables = Set<AnyCancellable>()
    
    init() {
        if let savedURL = UserDefaults.standard.string(forKey: "serverURL") {
            serverURL = savedURL
        }
        testConnection { _ in }
    }
    
    // MARK: - Configuration
    
    func updateServerURL(_ url: String) {
        serverURL = url
        UserDefaults.standard.set(url, forKey: "serverURL")
    }
    
    // MARK: - Connection Testing
    
    func testConnection(completion: @escaping (Bool) -> Void) {
        guard let url = URL(string: "\(serverURL)/api/scans") else {
            completion(false)
            return
        }
        
        var request = URLRequest(url: url)
        request.httpMethod = "GET"
        request.timeoutInterval = 5
        
        URLSession.shared.dataTask(with: request) { [weak self] data, response, error in
            DispatchQueue.main.async {
                if let httpResponse = response as? HTTPURLResponse,
                   httpResponse.statusCode == 200 {
                    self?.isConnected = true
                    completion(true)
                } else {
                    self?.isConnected = false
                    completion(false)
                }
            }
        }.resume()
    }
    
    // MARK: - Upload Scan
    
    func uploadScan(_ scanData: ScanData, completion: @escaping (Result<UploadResponse, Error>) -> Void) {
        guard let url = URL(string: "\(serverURL)/api/upload_lidar") else {
            completion(.failure(NetworkError.invalidURL))
            return
        }
        
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        request.timeoutInterval = 30
        
        do {
            let jsonData = try JSONEncoder().encode(scanData)
            request.httpBody = jsonData
            
            print("📤 Uploading scan with \(scanData.points.count) points...")
            
            URLSession.shared.dataTask(with: request) { data, response, error in
                if let error = error {
                    print("❌ Upload error: \(error.localizedDescription)")
                    completion(.failure(error))
                    return
                }
                
                guard let httpResponse = response as? HTTPURLResponse else {
                    completion(.failure(NetworkError.invalidResponse))
                    return
                }
                
                print("📥 Server response: \(httpResponse.statusCode)")
                
                guard httpResponse.statusCode == 200 else {
                    completion(.failure(NetworkError.serverError(statusCode: httpResponse.statusCode)))
                    return
                }
                
                guard let data = data else {
                    completion(.failure(NetworkError.noData))
                    return
                }
                
                do {
                    let response = try JSONDecoder().decode(UploadResponse.self, from: data)
                    print("✅ Upload successful! Scan ID: \(response.scanId)")
                    completion(.success(response))
                } catch {
                    print("❌ Decoding error: \(error)")
                    completion(.failure(error))
                }
            }.resume()
            
        } catch {
            completion(.failure(error))
        }
    }
    
    // MARK: - Fetch Scans
    
    func fetchScans(completion: @escaping (Result<[ScanRecord], Error>) -> Void) {
        guard let url = URL(string: "\(serverURL)/api/scans") else {
            completion(.failure(NetworkError.invalidURL))
            return
        }
        
        var request = URLRequest(url: url)
        request.httpMethod = "GET"
        request.timeoutInterval = 10
        
        URLSession.shared.dataTask(with: request) { data, response, error in
            if let error = error {
                completion(.failure(error))
                return
            }
            
            guard let data = data else {
                completion(.failure(NetworkError.noData))
                return
            }
            
            do {
                let response = try JSONDecoder().decode(ScansResponse.self, from: data)
                completion(.success(response.scans))
            } catch {
                completion(.failure(error))
            }
        }.resume()
    }
    
    // MARK: - Fetch Specific Scan
    
    func fetchScan(id: Int, completion: @escaping (Result<ScanDetail, Error>) -> Void) {
        guard let url = URL(string: "\(serverURL)/api/scan/\(id)") else {
            completion(.failure(NetworkError.invalidURL))
            return
        }
        
        var request = URLRequest(url: url)
        request.httpMethod = "GET"
        request.timeoutInterval = 10
        
        URLSession.shared.dataTask(with: request) { data, response, error in
            if let error = error {
                completion(.failure(error))
                return
            }
            
            guard let data = data else {
                completion(.failure(NetworkError.noData))
                return
            }
            
            do {
                let scanDetail = try JSONDecoder().decode(ScanDetail.self, from: data)
                completion(.success(scanDetail))
            } catch {
                completion(.failure(error))
            }
        }.resume()
    }
    
    // MARK: - Request Analysis
    
    func requestAnalysis(scanId: Int, completion: @escaping (Result<AnalysisResponse, Error>) -> Void) {
        guard let url = URL(string: "\(serverURL)/api/analyze/\(scanId)") else {
            completion(.failure(NetworkError.invalidURL))
            return
        }
        
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.timeoutInterval = 30
        
        URLSession.shared.dataTask(with: request) { data, response, error in
            if let error = error {
                completion(.failure(error))
                return
            }
            
            guard let data = data else {
                completion(.failure(NetworkError.noData))
                return
            }
            
            do {
                let analysis = try JSONDecoder().decode(AnalysisResponse.self, from: data)
                completion(.success(analysis))
            } catch {
                completion(.failure(error))
            }
        }.resume()
    }
    
    // MARK: - Delete Scan
    
    func deleteScan(id: Int, completion: @escaping (Result<Bool, Error>) -> Void) {
        guard let url = URL(string: "\(serverURL)/api/scan/\(id)") else {
            completion(.failure(NetworkError.invalidURL))
            return
        }
        
        var request = URLRequest(url: url)
        request.httpMethod = "DELETE"
        request.timeoutInterval = 10
        
        URLSession.shared.dataTask(with: request) { data, response, error in
            if let error = error {
                completion(.failure(error))
                return
            }
            
            guard let httpResponse = response as? HTTPURLResponse else {
                completion(.failure(NetworkError.invalidResponse))
                return
            }
            
            if httpResponse.statusCode == 200 {
                completion(.success(true))
            } else {
                completion(.failure(NetworkError.serverError(statusCode: httpResponse.statusCode)))
            }
        }.resume()
    }
}

// MARK: - Network Errors

enum NetworkError: LocalizedError {
    case invalidURL
    case invalidResponse
    case noData
    case serverError(statusCode: Int)
    case decodingError(String)
    case uploadFailed(String)
    
    var errorDescription: String? {
        switch self {
        case .invalidURL:
            return "Invalid server URL. Please check your settings."
        case .invalidResponse:
            return "Invalid response from server."
        case .noData:
            return "No data received from server."
        case .serverError(let statusCode):
            return "Server error with status code: \(statusCode)"
        case .decodingError(let message):
            return "Failed to decode response: \(message)"
        case .uploadFailed(let message):
            return "Upload failed: \(message)"
        }
    }
}

// MARK: - Response Models

struct UploadResponse: Codable {
    let success: Bool
    let scanId: Int
    let analysis: String
    
    enum CodingKeys: String, CodingKey {
        case success
        case scanId = "scan_id"
        case analysis
    }
    
    init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        success = try container.decode(Bool.self, forKey: .success)
        scanId = try container.decode(Int.self, forKey: .scanId)
        
        // Handle analysis - could be string or nested object
        if let analysisDict = try? container.decode([String: String].self, forKey: .analysis) {
            analysis = analysisDict.values.joined(separator: "\n")
        } else {
            analysis = try container.decode(String.self, forKey: .analysis)
        }
    }
}

struct ScansResponse: Codable {
    let scans: [ScanRecord]
}

struct ScanDetail: Codable {
    let id: Int
    let timestamp: String
    let pointCount: Int
    let data: ProcessedData
    
    enum CodingKeys: String, CodingKey {
        case id
        case timestamp
        case pointCount = "point_count"
        case data
    }
}

struct ProcessedData: Codable {
    let pointCount: Int
    let centroid: [Double]
    let boundingBox: BoundingBox
    let sampledPoints: [[Double]]
    let ranges: Ranges?
    let density: Double?
    
    enum CodingKeys: String, CodingKey {
        case pointCount = "point_count"
        case centroid
        case boundingBox = "bounding_box"
        case sampledPoints = "sampled_points"
        case ranges
        case density
    }
}

struct Ranges: Codable {
    let x: [Double]
    let y: [Double]
    let z: [Double]
}

struct BoundingBox: Codable {
    let min: [Double]
    let max: [Double]
    let dimensions: [Double]
}

struct AnalysisResponse: Codable {
    let scanId: Int
    let analysis: AnalysisResult
    
    enum CodingKeys: String, CodingKey {
        case scanId = "scan_id"
        case analysis
    }
}

struct AnalysisResult: Codable {
    let rawAnalysis: String
    let confidence: Double
    let timestamp: String?
    let structured: StructuredAnalysis?
    let fallbackAnalysis: FallbackAnalysis?
    
    enum CodingKeys: String, CodingKey {
        case rawAnalysis = "raw_analysis"
        case confidence
        case timestamp
        case structured
        case fallbackAnalysis = "fallback_analysis"
    }
    
    var displayText: String {
        if let structured = structured {
            return """
            \(structured.description ?? "")
            
            Quality: \(structured.quality ?? "N/A")
            
            Recommendations:
            \(structured.recommendations?.joined(separator: "\n• ") ?? "None")
            """
        } else if let fallback = fallbackAnalysis {
            return """
            \(fallback.description)
            Quality: \(fallback.quality)
            """
        } else {
            return rawAnalysis
        }
    }
}

struct StructuredAnalysis: Codable {
    let description: String?
    let quality: String?
    let recommendations: [String]?
    let issues: [String]?
}

struct FallbackAnalysis: Codable {
    let description: String
    let quality: String
    let recommendations: [String]
    let issues: [String]
}

// MARK: - WebSocket Manager (Optional for real-time updates)

class WebSocketManager: ObservableObject {
    @Published var isConnected = false
    @Published var latestScan: ScanDetail?
    
    private var webSocketTask: URLSessionWebSocketTask?
    private var serverURL: String
    
    init(serverURL: String) {
        self.serverURL = serverURL
    }
    
    func connect() {
        guard let url = URL(string: serverURL.replacingOccurrences(of: "http", with: "ws")) else {
            return
        }
        
        let session = URLSession(configuration: .default)
        webSocketTask = session.webSocketTask(with: url)
        webSocketTask?.resume()
        
        receiveMessage()
        isConnected = true
    }
    
    func disconnect() {
        webSocketTask?.cancel(with: .goingAway, reason: nil)
        isConnected = false
    }
    
    private func receiveMessage() {
        webSocketTask?.receive { [weak self] result in
            switch result {
            case .success(let message):
                switch message {
                case .string(let text):
                    print("📨 Received: \(text)")
                    self?.handleMessage(text)
                case .data(let data):
                    print("📨 Received data: \(data.count) bytes")
                @unknown default:
                    break
                }
                
                // Continue receiving
                self?.receiveMessage()
                
            case .failure(let error):
                print("❌ WebSocket error: \(error)")
                self?.isConnected = false
            }
        }
    }
    
    private func handleMessage(_ message: String) {
        // Parse incoming scan data
        if let data = message.data(using: .utf8),
           let scan = try? JSONDecoder().decode(ScanDetail.self, from: data) {
            DispatchQueue.main.async {
                self.latestScan = scan
            }
        }
    }
    
    func send(_ message: String) {
        let message = URLSessionWebSocketTask.Message.string(message)
        webSocketTask?.send(message) { error in
            if let error = error {
                print("❌ Send error: \(error)")
            }
        }
    }
}
