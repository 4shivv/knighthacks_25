//
//  LidarCaptureView.swift
//  LidarCapture
//
//  Created by nicky on 10/25/25.
//

import SwiftUI
import ARKit
import RealityKit

struct LidarCaptureView: View {
    @ObservedObject var networkManager: NetworkManager
    @StateObject private var arViewModel = ARViewModel()
    @State private var isScanning = false
    @State private var showingAlert = false
    @State private var alertMessage = ""
    @State private var pointCount = 0
    
    var body: some View {
        NavigationView {
            ZStack {
                // AR View
                ARViewContainer(arViewModel: arViewModel)
                    .edgesIgnoringSafeArea(.all)
                
                // Overlay UI
                VStack {
                    // Top Stats
                    HStack {
                        VStack(alignment: .leading, spacing: 4) {
                            Text("Points Captured")
                                .font(.caption)
                                .foregroundColor(.white)
                            Text("\(pointCount)")
                                .font(.title)
                                .fontWeight(.bold)
                                .foregroundColor(.white)
                        }
                        .padding()
                        .background(Color.black.opacity(0.6))
                        .cornerRadius(12)
                        
                        Spacer()
                        
                        VStack(alignment: .trailing, spacing: 4) {
                            Text("Status")
                                .font(.caption)
                                .foregroundColor(.white)
                            Text(isScanning ? "Scanning..." : "Ready")
                                .font(.headline)
                                .foregroundColor(isScanning ? .green : .white)
                        }
                        .padding()
                        .background(Color.black.opacity(0.6))
                        .cornerRadius(12)
                    }
                    .padding()
                    
                    Spacer()
                    
                    // Bottom Controls
                    VStack(spacing: 20) {
                        // Scan Button
                        Button(action: toggleScanning) {
                            ZStack {
                                Circle()
                                    .fill(isScanning ? Color.red : Color.purple)
                                    .frame(width: 80, height: 80)
                                    .shadow(radius: 10)
                                
                                Image(systemName: isScanning ? "stop.fill" : "camera.fill")
                                    .font(.system(size: 35))
                                    .foregroundColor(.white)
                            }
                        }
                        
                        HStack(spacing: 15) {
                            // Clear Button
                            Button(action: clearScan) {
                                Label("Clear", systemImage: "trash")
                                    .font(.headline)
                                    .foregroundColor(.white)
                                    .frame(maxWidth: .infinity)
                                    .padding()
                                    .background(Color.black.opacity(0.7))
                                    .cornerRadius(12)
                            }
                            
                            // Upload Button
                            Button(action: uploadScan) {
                                Label("Upload", systemImage: "icloud.and.arrow.up")
                                    .font(.headline)
                                    .foregroundColor(.white)
                                    .frame(maxWidth: .infinity)
                                    .padding()
                                    .background(Color.purple.opacity(0.9))
                                    .cornerRadius(12)
                            }
                            .disabled(pointCount == 0)
                            .opacity(pointCount == 0 ? 0.5 : 1.0)
                        }
                        .padding(.horizontal)
                    }
                    .padding(.bottom, 40)
                }
            }
            .navigationBarHidden(true)
            .alert(isPresented: $showingAlert) {
                Alert(title: Text("Upload Status"),
                      message: Text(alertMessage),
                      dismissButton: .default(Text("OK")))
            }
            .onReceive(arViewModel.$currentPointCount) { count in
                pointCount = count
            }
        }
    }
    
    private func toggleScanning() {
        if isScanning {
            arViewModel.stopScanning()
        } else {
            arViewModel.startScanning()
        }
        isScanning.toggle()
    }
    
    private func clearScan() {
        arViewModel.clearPoints()
        pointCount = 0
    }
    
    private func uploadScan() {
        guard let scanData = arViewModel.getScanData() else {
            alertMessage = "❌ No scan data available"
            showingAlert = true
            return
        }
        
        networkManager.uploadScan(scanData) { result in
            DispatchQueue.main.async {
                switch result {
                case .success(let response):
                    alertMessage = "✅ Upload successful!\nScan ID: \(response.scanId)"
                    if !response.analysis.isEmpty {
                        alertMessage += "\n\nAI Analysis:\n\(response.analysis)"
                    }
                case .failure(let error):
                    alertMessage = "❌ Upload failed: \(error.localizedDescription)"
                }
                showingAlert = true
            }
        }
    }
}

// AR View Container
struct ARViewContainer: UIViewRepresentable {
    @ObservedObject var arViewModel: ARViewModel
    
    func makeUIView(context: Context) -> ARView {
        let arView = ARView(frame: .zero)
        
        // Configure AR session
        let configuration = ARWorldTrackingConfiguration()
        configuration.sceneReconstruction = .mesh
        configuration.frameSemantics = .sceneDepth
        
        arView.session.run(configuration)
        arView.session.delegate = context.coordinator
        
        arViewModel.arView = arView
        
        return arView
    }
    
    func updateUIView(_ uiView: ARView, context: Context) {}
    
    func makeCoordinator() -> Coordinator {
        Coordinator(arViewModel: arViewModel)
    }
    
    class Coordinator: NSObject, ARSessionDelegate {
        let arViewModel: ARViewModel
        
        init(arViewModel: ARViewModel) {
            self.arViewModel = arViewModel
        }
        
        func session(_ session: ARSession, didUpdate frame: ARFrame) {
            arViewModel.processFrame(frame)
        }
    }
}

// AR View Model
class ARViewModel: ObservableObject {
    @Published var currentPointCount = 0
    @Published var isScanning = false
    
    var arView: ARView?
    private var capturedPoints: [LidarPoint] = []
    private var pointCloudEntity: Entity?
    
    func startScanning() {
        isScanning = true
        capturedPoints.removeAll()
        currentPointCount = 0
    }
    
    func stopScanning() {
        isScanning = false
    }
    
    func clearPoints() {
        capturedPoints.removeAll()
        currentPointCount = 0
        pointCloudEntity?.removeFromParent()
    }
    
    func processFrame(_ frame: ARFrame) {
        guard isScanning else { return }
        guard let depthData = frame.sceneDepth?.depthMap else { return }
        
        // Extract point cloud from depth data
        let points = extractPointsFromDepth(depthData, frame: frame)
        capturedPoints.append(contentsOf: points)
        
        DispatchQueue.main.async {
            self.currentPointCount = self.capturedPoints.count
        }
    }
    
    private func extractPointsFromDepth(_ depthMap: CVPixelBuffer, frame: ARFrame) -> [LidarPoint] {
        var points: [LidarPoint] = []
        
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
        
        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        
        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else {
            return points
        }
        
        let floatBuffer = baseAddress.assumingMemoryBound(to: Float32.self)
        
        // Sample every Nth pixel for performance
        let step = 10
        
        for y in stride(from: 0, to: height, by: step) {
            for x in stride(from: 0, to: width, by: step) {
                let index = y * width + x
                let depth = floatBuffer[index]
                
                // Skip invalid depths
                guard depth > 0 && depth < 10 else { continue }
                
                // Convert pixel to 3D point
                let normalizedPoint = CGPoint(
                    x: CGFloat(x) / CGFloat(width),
                    y: CGFloat(y) / CGFloat(height)
                )
                
                let worldPoint = frame.camera.unprojectPoint(
                    normalizedPoint,
                    ontoPlaneAt: depth,
                    orientation: .portrait
                )
                
                points.append(LidarPoint(
                    x: Double(worldPoint.x),
                    y: Double(worldPoint.y),
                    z: Double(worldPoint.z)
                ))
            }
        }
        
        return points
    }
    
    func getScanData() -> ScanData? {
        guard !capturedPoints.isEmpty else { return nil }
        
        return ScanData(
            points: capturedPoints,
            timestamp: ISO8601DateFormatter().string(from: Date()),
            deviceInfo: [
                "model": UIDevice.current.model,
                "name": UIDevice.current.name,
                "systemVersion": UIDevice.current.systemVersion
            ]
        )
    }
}

// Extension for camera unprojection
extension ARCamera {
    func unprojectPoint(_ point: CGPoint, ontoPlaneAt depth: Float, orientation: UIInterfaceOrientation) -> simd_float3 {
        let viewMatrix = self.viewMatrix(for: orientation)
        let projectionMatrix = self.projectionMatrix(for: orientation, viewportSize: CGSize(width: 1, height: 1), zNear: 0.001, zFar: 1000)
        
        // Convert normalized point to NDC
        let ndcPoint = simd_float4(
            Float(point.x) * 2 - 1,
            Float(1 - point.y) * 2 - 1,
            depth,
            1
        )
        
        // Inverse projection
        let invProjection = projectionMatrix.inverse
        let invView = viewMatrix.inverse
        
        var worldPoint = invProjection * ndcPoint
        worldPoint = worldPoint / worldPoint.w
        worldPoint = invView * worldPoint
        
        return simd_float3(worldPoint.x, worldPoint.y, worldPoint.z)
    }
}

struct LidarCaptureView_Previews: PreviewProvider {
    static var previews: some View {
        LidarCaptureView(networkManager: NetworkManager())
    }
}
