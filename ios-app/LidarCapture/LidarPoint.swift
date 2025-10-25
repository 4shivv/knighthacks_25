//
//  LidarPoint.swift
//  LidarCapture
//
//  Created by nicky on 10/25/25.
//

import Foundation


struct LidarPoint: Codable, Hashable {
    let x: Double
    let y: Double
    let z: Double
    
    init(x: Double, y: Double, z: Double) {
        self.x = x
        self.y = y
        self.z = z
    }
    
    // Convert to array for JSON serialization
    var asArray: [Double] {
        return [x, y, z]
    }
    
    // Distance from origin
    var magnitude: Double {
        return sqrt(x * x + y * y + z * z)
    }
    
    // Distance to another point
    func distance(to other: LidarPoint) -> Double {
        let dx = x - other.x
        let dy = y - other.y
        let dz = z - other.z
        return sqrt(dx * dx + dy * dy + dz * dz)
    }
}

// MARK: - ScanData Model

struct ScanData: Codable {
    let points: [LidarPoint]
    let timestamp: String
    let deviceInfo: [String: String]
    
    init(points: [LidarPoint], timestamp: String, deviceInfo: [String: String]) {
        self.points = points
        self.timestamp = timestamp
        self.deviceInfo = deviceInfo
    }
    
    // Custom encoding to match Flask API format
    enum CodingKeys: String, CodingKey {
        case points
        case timestamp
        case deviceInfo = "device_info"
    }
    
    func encode(to encoder: Encoder) throws {
        var container = encoder.container(keyedBy: CodingKeys.self)
        
        // Encode points as array of arrays [[x, y, z], [x, y, z], ...]
        let pointsArray = points.map { [$0.x, $0.y, $0.z] }
        try container.encode(pointsArray, forKey: .points)
        
        try container.encode(timestamp, forKey: .timestamp)
        try container.encode(deviceInfo, forKey: .deviceInfo)
    }
    
    // Statistics
    var pointCount: Int {
        return points.count
    }
    
    var centroid: LidarPoint {
        guard !points.isEmpty else {
            return LidarPoint(x: 0, y: 0, z: 0)
        }
        
        let sumX = points.reduce(0.0) { $0 + $1.x }
        let sumY = points.reduce(0.0) { $0 + $1.y }
        let sumZ = points.reduce(0.0) { $0 + $1.z }
        
        let count = Double(points.count)
        return LidarPoint(x: sumX / count, y: sumY / count, z: sumZ / count)
    }
    
    var boundingBox: (min: LidarPoint, max: LidarPoint) {
        guard !points.isEmpty else {
            return (LidarPoint(x: 0, y: 0, z: 0), LidarPoint(x: 0, y: 0, z: 0))
        }
        
        var minX = Double.infinity
        var minY = Double.infinity
        var minZ = Double.infinity
        var maxX = -Double.infinity
        var maxY = -Double.infinity
        var maxZ = -Double.infinity
        
        for point in points {
            minX = min(minX, point.x)
            minY = min(minY, point.y)
            minZ = min(minZ, point.z)
            maxX = max(maxX, point.x)
            maxY = max(maxY, point.y)
            maxZ = max(maxZ, point.z)
        }
        
        return (
            LidarPoint(x: minX, y: minY, z: minZ),
            LidarPoint(x: maxX, y: maxY, z: maxZ)
        )
    }
}

// MARK: - Point Cloud Statistics

struct PointCloudStats {
    let pointCount: Int
    let centroid: LidarPoint
    let boundingBox: (min: LidarPoint, max: LidarPoint)
    let dimensions: LidarPoint
    let density: Double
    
    init(from scanData: ScanData) {
        self.pointCount = scanData.pointCount
        self.centroid = scanData.centroid
        self.boundingBox = scanData.boundingBox
        
        let width = boundingBox.max.x - boundingBox.min.x
        let height = boundingBox.max.y - boundingBox.min.y
        let depth = boundingBox.max.z - boundingBox.min.z
        
        self.dimensions = LidarPoint(x: width, y: height, z: depth)
        
        let volume = width * height * depth
        self.density = volume > 0 ? Double(pointCount) / volume : 0
    }
    
    var description: String {
        return """
        Point Cloud Statistics:
        - Points: \(pointCount)
        - Centroid: (\(String(format: "%.2f", centroid.x)), \(String(format: "%.2f", centroid.y)), \(String(format: "%.2f", centroid.z)))
        - Dimensions: \(String(format: "%.2f", dimensions.x))m × \(String(format: "%.2f", dimensions.y))m × \(String(format: "%.2f", dimensions.z))m
        - Density: \(String(format: "%.2f", density)) points/m³
        """
    }
}

// MARK: - Point Cloud Utilities

extension Array where Element == LidarPoint {
    // Downsample point cloud
    func downsample(targetCount: Int) -> [LidarPoint] {
        guard count > targetCount else { return self }
        
        let step = count / targetCount
        return stride(from: 0, to: count, by: step).map { self[$0] }
    }
    
    // Remove outliers using statistical method
    func removeOutliers(standardDeviations: Double = 2.0) -> [LidarPoint] {
        guard count > 3 else { return self }
        
        // Calculate mean distance from centroid
        let centroid = LidarPoint(
            x: map { $0.x }.reduce(0, +) / Double(count),
            y: map { $0.y }.reduce(0, +) / Double(count),
            z: map { $0.z }.reduce(0, +) / Double(count)
        )
        
        let distances = map { $0.distance(to: centroid) }
        let meanDistance = distances.reduce(0, +) / Double(count)
        
        let variance = distances.map { pow($0 - meanDistance, 2) }.reduce(0, +) / Double(count)
        let stdDev = sqrt(variance)
        
        let threshold = meanDistance + (standardDeviations * stdDev)
        
        return zip(self, distances)
            .filter { $0.1 <= threshold }
            .map { $0.0 }
    }
    
    // Filter by distance range
    func filter(minDistance: Double, maxDistance: Double) -> [LidarPoint] {
        return filter { point in
            let distance = point.magnitude
            return distance >= minDistance && distance <= maxDistance
        }
    }
}
