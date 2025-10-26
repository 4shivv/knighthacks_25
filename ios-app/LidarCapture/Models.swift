//
//  Models.swift
//  LidarCapture
//
//  Created by nicky on 10/25/25.
//


import Foundation

// Response wrapper for /api/scans
struct ScansResponse: Codable {
    let scans: [ScanRecord]
}

// Single scan record used in lists/history
struct ScanRecord: Identifiable, Codable {
    let id: Int
    let timestamp: String
    let pointCount: Int

    enum CodingKeys: String, CodingKey {
        case id
        case timestamp
        case pointCount = "point_count"   // map server snake_case → camelCase
    }

    // Convenience formatter for UI
    var formattedDate: String {
        let iso = ISO8601DateFormatter()
        if let date = iso.date(from: timestamp) {
            let df = DateFormatter()
            df.dateStyle = .short
            df.timeStyle = .short
            return df.string(from: date)
        }
        return timestamp
    }
}
