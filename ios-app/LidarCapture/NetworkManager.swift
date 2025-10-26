import Foundation
import Combine

class NetworkManager: ObservableObject {
    @Published var isConnected = false
    @Published var pointCloudDensity = 5
    
    private var serverURL: String = "http://192.168.1.153:5000"
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
        
        URLSession.shared.dataTask(with: request) { [weak self] _, response, _ in
            DispatchQueue.main.async {
                if let httpResponse = response as? HTTPURLResponse, httpResponse.statusCode == 200 {
                    self?.isConnected = true
                    completion(true)
                } else {
                    self?.isConnected = false
                    completion(false)
                }
            }
        }.resume()
    }
    
    // MARK: - Upload Camera Frame (Binary JPEG) - FIXED
    
    func uploadFrame(_ imageData: Data, completion: @escaping (Result<Bool, Error>) -> Void) {
        guard let url = URL(string: "\(serverURL)/api/upload_frame") else {
            completion(.failure(NetworkError.invalidURL))
            return
        }
        
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("image/jpeg", forHTTPHeaderField: "Content-Type")
        request.timeoutInterval = 10 // Increased timeout
        request.cachePolicy = .reloadIgnoringLocalCacheData
        
        // Use uploadTask for better performance
        URLSession.shared.uploadTask(with: request, from: imageData) { _, response, error in
            if let error = error {
                // Silently ignore timeout errors to reduce log spam
                if (error as NSError).code != NSURLErrorTimedOut {
                    print("❌ Camera frame error: \(error.localizedDescription)")
                }
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
    
    // MARK: - Upload LiDAR Frame (Binary JPEG) - FIXED
    
    func uploadLidarFrame(points: [LidarPoint], completion: @escaping (Bool) -> Void) {
        guard let url = URL(string: "\(serverURL)/api/upload_lidar_frame") else {
            print("❌ Invalid LiDAR upload URL")
            completion(false)
            return
        }

        // Convert array of LidarPoint to JSON
        let payload: [String: Any] = [
            "points": points.map { ["x": $0.x, "y": $0.y, "z": $0.z] }
        ]

        guard let jsonData = try? JSONSerialization.data(withJSONObject: payload, options: []) else {
            print("❌ Failed to encode LiDAR points to JSON")
            completion(false)
            return
        }

        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        request.timeoutInterval = 10
        request.httpBody = jsonData

        let task = URLSession.shared.dataTask(with: request) { _, response, error in
            if let error = error {
                print("❌ LiDAR upload error: \(error.localizedDescription)")
                completion(false)
                return
            }

            guard let httpResponse = response as? HTTPURLResponse else {
                print("❌ Invalid LiDAR server response")
                completion(false)
                return
            }

            if httpResponse.statusCode == 200 {
                print("📡 LiDAR JSON points uploaded successfully (\(points.count) points)")
                completion(true)
            } else {
                print("⚠️ LiDAR server returned \(httpResponse.statusCode)")
                completion(false)
            }
        }

        task.resume()
    }

    // MARK: - Upload Scan (Full Point Cloud)
    
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
        
        // Handle analysis - could be string, dictionary, or complex object
        if let analysisString = try? container.decode(String.self, forKey: .analysis) {
            analysis = analysisString
        } else if let analysisDict = try? container.decode([String: AnyCodable].self, forKey: .analysis) {
            // Convert dictionary to formatted string
            var parts: [String] = []
            
            if let rawAnalysis = analysisDict["raw_analysis"]?.value as? String {
                parts.append(rawAnalysis)
            }
            
            if let confidence = analysisDict["confidence"]?.value as? Double {
                parts.append("Confidence: \(String(format: "%.1f%%", confidence * 100))")
            }
            
            if let structured = analysisDict["structured"]?.value as? [String: Any] {
                if let description = structured["description"] as? String {
                    parts.append("\nDescription: \(description)")
                }
                if let quality = structured["quality"] as? String {
                    parts.append("Quality: \(quality)")
                }
            }
            
            if let fallback = analysisDict["fallback_analysis"]?.value as? [String: Any] {
                if let description = fallback["description"] as? String {
                    parts.append(description)
                }
                if let quality = fallback["quality"] as? String {
                    parts.append("Quality: \(quality)")
                }
            }
            
            analysis = parts.isEmpty ? "Analysis completed" : parts.joined(separator: "\n")
        } else {
            // Fallback: try to decode as any JSON and convert to string
            if let anyValue = try? container.decode(AnyCodable.self, forKey: .analysis) {
                analysis = String(describing: anyValue.value)
            } else {
                analysis = "Analysis completed"
            }
        }
    }
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

// Helper to decode unknown JSON types
struct AnyCodable: Codable {
    let value: Any
    
    init(_ value: Any) {
        self.value = value
    }
    
    init(from decoder: Decoder) throws {
        let container = try decoder.singleValueContainer()
        
        if let bool = try? container.decode(Bool.self) {
            value = bool
        } else if let int = try? container.decode(Int.self) {
            value = int
        } else if let double = try? container.decode(Double.self) {
            value = double
        } else if let string = try? container.decode(String.self) {
            value = string
        } else if let array = try? container.decode([AnyCodable].self) {
            value = array.map { $0.value }
        } else if let dictionary = try? container.decode([String: AnyCodable].self) {
            value = dictionary.mapValues { $0.value }
        } else {
            value = NSNull()
        }
    }
    
    func encode(to encoder: Encoder) throws {
        var container = encoder.singleValueContainer()
        
        switch value {
        case let bool as Bool:
            try container.encode(bool)
        case let int as Int:
            try container.encode(int)
        case let double as Double:
            try container.encode(double)
        case let string as String:
            try container.encode(string)
        case let array as [Any]:
            try container.encode(array.map { AnyCodable($0) })
        case let dictionary as [String: Any]:
            try container.encode(dictionary.mapValues { AnyCodable($0) })
        default:
            try container.encodeNil()
        }
    }
}
