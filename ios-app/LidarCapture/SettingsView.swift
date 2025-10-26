import SwiftUI

struct SettingsView: View {
    @EnvironmentObject var networkManager: NetworkManager
    @AppStorage("serverURL") private var serverURL = "http://192.168.1.134:5000"
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
                alertMessage = success
                    ? "✅ Connection successful!"
                    : "❌ Connection failed. Check your server URL."
                showingAlert = true
            }
        }
    }
}

