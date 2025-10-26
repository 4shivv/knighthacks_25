import SwiftUI
import ARKit
import RealityKit
import AVFoundation
import simd

struct LidarCaptureView: View {
    @EnvironmentObject var networkManager: NetworkManager
    @StateObject private var arViewModel = ARViewModel()

    @State private var isScanning = false
    @State private var isStreaming = false
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
                    // Top HUD
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
                            HStack(spacing: 4) {
                                if isStreaming {
                                    Circle().fill(Color.red)
                                        .frame(width: 8, height: 8)
                                    Text("LIVE")
                                        .font(.caption)
                                        .fontWeight(.bold)
                                        .foregroundColor(.red)
                                }
                                Text(isScanning ? "Scanning..." : "Ready")
                                    .font(.headline)
                                    .foregroundColor(isScanning ? .green : .white)
                            }
                        }
                        .padding()
                        .background(Color.black.opacity(0.6))
                        .cornerRadius(12)
                    }
                    .padding()

                    Spacer()

                    // Bottom Controls
                    VStack(spacing: 20) {
                        // Stream Toggle
                        HStack {
                            Text("Live Stream")
                                .font(.headline)
                                .foregroundColor(.white)
                            Toggle("", isOn: $isStreaming)
                                .labelsHidden()
                                .onChange(of: isStreaming) { newValue in
                                    toggleStreaming(newValue)
                                }
                        }
                        .padding()
                        .background(Color.black.opacity(0.7))
                        .cornerRadius(12)
                        .padding(.horizontal)

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
                            Button(action: clearScan) {
                                Label("Clear", systemImage: "trash")
                                    .font(.headline)
                                    .foregroundColor(.white)
                                    .frame(maxWidth: .infinity)
                                    .padding()
                                    .background(Color.black.opacity(0.7))
                                    .cornerRadius(12)
                            }

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
        if isScanning { arViewModel.stopScanning() }
        else { arViewModel.startScanning() }
        isScanning.toggle()
    }

    private func toggleStreaming(_ enabled: Bool) {
        if enabled {
            arViewModel.startStreaming(networkManager: networkManager)
        } else {
            arViewModel.stopStreaming()
        }
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

// MARK: - ARViewContainer

struct ARViewContainer: UIViewRepresentable {
    @ObservedObject var arViewModel: ARViewModel

    func makeUIView(context: Context) -> ARView {
        let arView = ARView(frame: .zero)
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
        init(arViewModel: ARViewModel) { self.arViewModel = arViewModel }
        func session(_ session: ARSession, didUpdate frame: ARFrame) {
            arViewModel.processFrame(frame)
        }
    }
}

// MARK: - ARViewModel

class ARViewModel: ObservableObject {
    @Published var currentPointCount = 0
    @Published var isScanning = false
    @Published var isStreaming = false

    var arView: ARView?
    private var capturedPoints: [LidarPoint] = []
    private var streamTimer: Timer?
    private var networkManager: NetworkManager?
    private var frameCount = 0
    private var lastFrame: UIImage?

    func processFrame(_ frame: ARFrame) {
        if isScanning {
            guard let depthData = frame.sceneDepth?.depthMap else { return }
            let points = extractPointsFromDepth(depthData, frame: frame)
            capturedPoints.append(contentsOf: points)
            DispatchQueue.main.async {
                self.currentPointCount = self.capturedPoints.count
            }
        }
        updateFrameImage(frame)
    }

    private func updateFrameImage(_ frame: ARFrame) {
        let image = frame.capturedImage
        let ciImage = CIImage(cvPixelBuffer: image)
        let context = CIContext()
        if let cgImage = context.createCGImage(ciImage, from: ciImage.extent) {
            lastFrame = UIImage(cgImage: cgImage)
        }
    }

    func startScanning() {
        isScanning = true
        capturedPoints.removeAll()
        currentPointCount = 0
    }

    func stopScanning() { isScanning = false }
    func clearPoints() { capturedPoints.removeAll(); currentPointCount = 0 }

    func startStreaming(networkManager: NetworkManager) {
        self.networkManager = networkManager
        isStreaming = true
        streamTimer = Timer.scheduledTimer(withTimeInterval: 0.2, repeats: true) { [weak self] _ in
            self?.sendFrame()
        }
    }

    func stopStreaming() {
        isStreaming = false
        streamTimer?.invalidate()
        streamTimer = nil
        networkManager = nil
    }

    private func sendFrame() {
        guard let frame = lastFrame,
              let networkManager = networkManager,
              let imageData = frame.jpegData(compressionQuality: 0.6) else { return }
        frameCount += 1
        networkManager.uploadFrame(imageData) { result in
            switch result {
            case .success:
                print("📤 Sent frame #\(self.frameCount)")
            case .failure(let error):
                print("❌ Frame upload error: \(error.localizedDescription)")
            }
        }
    }

    private func extractPointsFromDepth(_ depthMap: CVPixelBuffer, frame: ARFrame) -> [LidarPoint] {
        var points: [LidarPoint] = []
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else { return [] }
        let floatBuffer = baseAddress.assumingMemoryBound(to: Float32.self)
        let step = 10

        for y in stride(from: 0, to: height, by: step) {
            for x in stride(from: 0, to: width, by: step) {
                let index = y * width + x
                let depth = floatBuffer[index]
                guard depth > 0 && depth < 10 else { continue }

                let normalized = CGPoint(x: CGFloat(x) / CGFloat(width),
                                         y: CGFloat(y) / CGFloat(height))
                let world = frame.camera.unprojectPoint(normalized,
                                                        ontoPlaneAt: depth,
                                                        orientation: .portrait)
                points.append(LidarPoint(x: Double(world.x),
                                         y: Double(world.y),
                                         z: Double(world.z)))
            }
        }
        return points
    }

    func getScanData() -> ScanData? {
        guard !capturedPoints.isEmpty else { return nil }
        return ScanData(points: capturedPoints,
                        timestamp: ISO8601DateFormatter().string(from: Date()),
                        deviceInfo: [
                            "model": UIDevice.current.model,
                            "name": UIDevice.current.name,
                            "systemVersion": UIDevice.current.systemVersion
                        ])
    }
}

// MARK: - ARCamera Extension (Fixes compile errors)

extension ARCamera {
    func unprojectPoint(_ point: CGPoint,
                        ontoPlaneAt depth: Float,
                        orientation: UIInterfaceOrientation) -> simd_float3 {
        let projectionMatrix = self.projectionMatrix(for: orientation,
                                                     viewportSize: CGSize(width: 1, height: 1),
                                                     zNear: 0.001, zFar: 1000)
        let viewMatrix = self.viewMatrix(for: orientation)
        let ndc = simd_float4(
            Float(point.x) * 2 - 1,
            Float(1 - point.y) * 2 - 1,
            depth,
            1.0
        )
        let invProjection = projectionMatrix.inverse
        let invView = viewMatrix.inverse
        var worldPoint = invProjection * ndc
        worldPoint /= worldPoint.w
        worldPoint = invView * worldPoint
        return simd_float3(worldPoint.x, worldPoint.y, worldPoint.z)
    }
}

// MARK: - Preview

struct LidarCaptureView_Previews: PreviewProvider {
    static var previews: some View {
        LidarCaptureView()
    }
}

