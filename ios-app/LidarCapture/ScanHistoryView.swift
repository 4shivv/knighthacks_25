//
//  ScanHistoryView.swift
//  LidarCapture
//
//  Created by nicky on 10/25/25.
//

import SwiftUI

struct ScanHistoryView: View {
    @EnvironmentObject var networkManager: NetworkManager
    @State private var scans: [ScanRecord] = []
    @State private var isLoading = false
    
    var body: some View {
        NavigationView {
            ZStack {
                if isLoading {
                    ProgressView("Loading scans...")
                } else if scans.isEmpty {
                    VStack {
                        Image(systemName: "tray")
                            .font(.system(size: 60))
                            .foregroundColor(.gray)
                        Text("No scans yet")
                            .font(.title2)
                            .foregroundColor(.gray)
                            .padding()
                    }
                } else {
                    List(scans) { scan in
                        ScanRowView(scan: scan)
                    }
                }
            }
            .navigationTitle("Scan History")
            .toolbar {
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button(action: loadScans) {
                        Image(systemName: "arrow.clockwise")
                    }
                }
            }
            .onAppear(perform: loadScans)
        }
    }
    
    private func loadScans() {
        isLoading = true
        networkManager.fetchScans { result in
            DispatchQueue.main.async {
                isLoading = false
                switch result {
                case .success(let fetchedScans):
                    self.scans = fetchedScans
                case .failure(let error):
                    print("❌ Error loading scans: \(error.localizedDescription)")
                }
            }
        }
    }
}

struct ScanRowView: View {
    let scan: ScanRecord
    
    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Image(systemName: "cube.fill")
                    .foregroundColor(.purple)
                Text("Scan #\(scan.id)")
                    .font(.headline)
                Spacer()
                Text(scan.formattedDate)
                    .font(.caption)
                    .foregroundColor(.gray)
            }
            
            HStack {
                Label("\(scan.pointCount)", systemImage: "point.3.filled.connected.trianglepath.dotted")
                    .font(.subheadline)
                    .foregroundColor(.secondary)
                Spacer()
            }
        }
        .padding(.vertical, 4)
    }
}
