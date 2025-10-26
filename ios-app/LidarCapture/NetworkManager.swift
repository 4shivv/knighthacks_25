import Foundation
import Combine

class NetworkManager: ObservableObject {
    @Published var isConnected = false
    @Published var pointCloudDensity = 5
    
    private var serverURL: String = "http://192.168.1.134:5000"
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
    
    // MARK: - Upload Frame (Live Streaming)
    
    func uploadFrame(_ imageData: Data, completion: @escaping (Result<Void, Error>) -> Void) {
        guard let url = URL(string: "\(serverURL)/api/upload_frame") else {
            completion(.failure(NetworkError.invalidURL))
            return
        }
        
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("image/jpeg", forHTTPHeaderField: "Content-Type")
        request.timeoutInterval = 10
        
        URLSession.shared.uploadTask(with: request, from: imageData) { _, response, error in
            if let error = error {
                completion(.failure(error))
                return
            }
            
            guard let httpResponse = response as? HTTPURLResponse else {
                completion(.failure(NetworkError.invalidResponse))
                return
            }
            
            guard httpResponse.statusCode == 200 else {
                completion(.failure(NetworkError.serverError(statusCode: httpResponse.statusCode)))
                return
            }
            
            completion(.success(()))
        }.resume()
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
        
        do {
            let jsonData = try JSONEncoder().encode(scanData)
            request.httpBody = jsonData
            
            URLSession.shared.dataTask(with: request) { data, response, error in
                if let error = error {
                    completion(.failure(error))
                    return
                }
                
                guard let httpResponse = response as? HTTPURLResponse, httpResponse.statusCode == 200,
                      let data = data else {
                    completion(.failure(NetworkError.invalidResponse))
                    return
                }
                
                do {
                    let decoded = try JSONDecoder().decode(UploadResponse.self, from: data)
                    completion(.success(decoded))
                } catch {
                    completion(.failure(error))
                }
            }.resume()
        } catch {
            completion(.failure(error))
        }
    }
    
    // MARK: - Fetch Scans (for History)
    
    func fetchScans(completion: @escaping (Result<[ScanRecord], Error>) -> Void) {
        guard let url = URL(string: "\(serverURL)/api/scans") else {
            completion(.failure(NetworkError.invalidURL))
            return
        }
        
        var request = URLRequest(url: url)
        request.httpMethod = "GET"
        request.timeoutInterval = 10
        
        URLSession.shared.dataTask(with: request) { data, _, error in
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
}

// MARK: - Errors and Models

enum NetworkError: LocalizedError {
    case invalidURL
    case invalidResponse
    case noData
    case serverError(statusCode: Int)
    
    var errorDescription: String? {
        switch self {
        case .invalidURL: return "Invalid URL"
        case .invalidResponse: return "Invalid response"
        case .noData: return "No data"
        case .serverError(let code): return "Server error: \(code)"
        }
    }
}

struct UploadResponse: Codable {
    let success: Bool
    let scanId: Int
    let analysis: String
}



