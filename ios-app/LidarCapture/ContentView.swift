//
//  ContentView.swift
//  LidarCapture
//
//  Created by nicky on 10/25/25.
//

import SwiftUI

struct ContentView: View {
    @StateObject private var networkManager = NetworkManager()
    @State private var selectedTab = 0
    
    var body: some View {
        TabView(selection: $selectedTab) {
            // LiDAR Capture Tab
            LidarCaptureView(networkManager: networkManager)
                .tabItem {
                    Label("Scan", systemImage: "camera.fill")
                }
                .tag(0)
            
            // History Tab
            ScanHistoryView(networkManager: networkManager)
                .tabItem {
                    Label("History", systemImage: "clock.fill")
                }
                .tag(1)
            
            // Settings Tab
            SettingsView(networkManager: networkManager)
                .tabItem {
                    Label("Settings", systemImage: "gearshape.fill")
                }
                .tag(2)
        }
        .accentColor(.purple)
    }
}

struct ScanHistoryView: View {
    @ObservedObject var networkManager: NetworkManager
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
                    print("Error loading scans: \(error)")
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

struct SettingsView: View {
    @ObservedObject var networkManager: NetworkManager
    @AppStorage("serverURL") private var serverURL = "http://localhost:5000"
    @AppStorage("autoUpload") private var autoUpload = true
    @State private var showingAlert = false
    @State private var alertMessage = ""
    
    var body: some View {
        NavigationView {
            Form {
                Section(header: Text("Server Configuration")) {
                    HStack {
                        Text("Status")
                        Spacer()
                        Circle()
                            .fill(networkManager.isConnected ? Color.green : Color.red)
                            .frame(width: 12, height: 12)
                        Text(networkManager.isConnected ? "Connected" : "Disconnected")
                            .font(.caption)
                            .foregroundColor(.gray)
                    }
                    
                    TextField("Server URL", text: $serverURL)
                        .autocapitalization(.none)
                        .keyboardType(.URL)
                    
                    Button("Test Connection") {
                        testConnection()
                    }
                }
                
                Section(header: Text("Scan Settings")) {
                    Toggle("Auto Upload", isOn: $autoUpload)
                    
                    Stepper(value: $networkManager.pointCloudDensity, in: 1...10) {
                        Text("Point Density: \(networkManager.pointCloudDensity)")
                    }
                }
                
                Section(header: Text("About")) {
                    HStack {
                        Text("Version")
                        Spacer()
                        Text("1.0.0")
                            .foregroundColor(.gray)
                    }
                    
                    HStack {
                        Text("Build")
                        Spacer()
                        Text("001")
                            .foregroundColor(.gray)
                    }
                }
            }
            .navigationTitle("Settings")
            .alert(isPresented: $showingAlert) {
                Alert(title: Text("Connection Test"),
                      message: Text(alertMessage),
                      dismissButton: .default(Text("OK")))
            }
        }
    }
    
    private func testConnection() {
        networkManager.updateServerURL(serverURL)
        networkManager.testConnection { success in
            DispatchQueue.main.async {
                alertMessage = success ? "Connection successful!" : " Connection failed. Check your server URL."
                showingAlert = true
            }
        }
    }
}

struct ScanRecord: Identifiable, Codable {
    let id: Int
    let timestamp: String
    let pointCount: Int
    
    var formattedDate: String {
        let formatter = ISO8601DateFormatter()
        if let date = formatter.date(from: timestamp) {
            let displayFormatter = DateFormatter()
            displayFormatter.dateStyle = .short
            displayFormatter.timeStyle = .short
            return displayFormatter.string(from: date)
        }
        return timestamp
    }
}

struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
    }
}
