import SwiftUI
import ARKit
import RealityKit
import AVFoundation

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
                ARViewContainer(arViewModel: arViewModel)
                    .edgesIgnoringSafeArea(.all)
                
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
                            HStack(spacing: 4) {
                                if isStreaming {
                                    Circle()
                                        .fill(Color.red)
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
            print("🎥 Starting Live Stream (Camera + LiDAR)")
            arViewModel.startStreaming(networkManager: networkManager)
        } else {
            print("🛑 Stopping Live Stream")
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

// MARK: - AR Container
struct ARViewContainer: UIViewRepresentable {
    @ObservedObject var arViewModel: ARViewModel
    func makeUIView(context: Context) -> ARView {
        let arView = ARView(frame: .zero)
        let config = ARWorldTrackingConfiguration()
        config.sceneReconstruction = .mesh
        config.frameSemantics = [.sceneDepth]
        arView.session.run(config)
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
    private var lastCameraFrame: UIImage?
    private var lastDepthFrame: UIImage?
    
    // MARK: - Scanning
    func startScanning() {
        isScanning = true
        capturedPoints.removeAll()
        currentPointCount = 0
    }
    func stopScanning() { isScanning = false }
    func clearPoints() {
        capturedPoints.removeAll()
        currentPointCount = 0
    }
    
    // MARK: - Streaming
    func startStreaming(networkManager: NetworkManager) {
        self.networkManager = networkManager
        isStreaming = true
        
        // Ensure LiDAR capture is active even if not scanning
        activateLidar()
        
        // Stream both LiDAR + Camera at 20 FPS
        streamTimer = Timer.scheduledTimer(withTimeInterval: 0.05, repeats: true) { [weak self] _ in
            self?.sendFrames()
        }
    }
    func stopStreaming() {
        isStreaming = false
        streamTimer?.invalidate()
        streamTimer = nil
        networkManager = nil
        deactivateLidar()
    }
    
    private func activateLidar() {
        guard let arView = arView else { return }
        let config = ARWorldTrackingConfiguration()
        config.sceneReconstruction = .mesh
        config.frameSemantics = [.sceneDepth]
        arView.session.run(config, options: [.resetTracking, .removeExistingAnchors])
        print("📡 LiDAR active for live stream")
    }
    private func deactivateLidar() {
        guard let arView = arView else { return }
        let config = ARWorldTrackingConfiguration()
        config.sceneReconstruction = .mesh
        arView.session.run(config, options: [.resetTracking, .removeExistingAnchors])
        print("🛑 LiDAR stopped")
    }
    
    func processFrame(_ frame: ARFrame) {
        // Capture points only when scanning
        if isScanning {
            guard let depthData = frame.sceneDepth?.depthMap else { return }
            let points = extractPointsFromDepth(depthData, frame: frame)
            capturedPoints.append(contentsOf: points)
            DispatchQueue.main.async { self.currentPointCount = self.capturedPoints.count }
        }
        
        // Always update frames if streaming
        if isStreaming {
            updateCameraFrame(frame)
            if let depthMap = frame.sceneDepth?.depthMap { updateDepthFrame(depthMap) }
        }
    }
    
    // MARK: - Frame Conversion
    private func updateCameraFrame(_ frame: ARFrame) {
        let ciImage = CIImage(cvPixelBuffer: frame.capturedImage)
        let context = CIContext()
        if let cgImage = context.createCGImage(ciImage, from: ciImage.extent) {
            lastCameraFrame = UIImage(cgImage: cgImage)
        }
    }
    private func updateDepthFrame(_ depthMap: CVPixelBuffer) {
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else { return }
        let floatBuffer = baseAddress.assumingMemoryBound(to: Float32.self)
        let rgbSpace = CGColorSpaceCreateDeviceRGB()
        guard let context = CGContext(
            data: nil, width: width, height: height, bitsPerComponent: 8,
            bytesPerRow: width * 4, space: rgbSpace,
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ) else { return }
        let pixels = context.data?.assumingMemoryBound(to: UInt8.self)
        for y in 0..<height {
            for x in 0..<width {
                let i = y * width + x
                let depth = floatBuffer[i]
                let norm = min(max(depth / 5.0, 0), 1)
                let (r, g, b) = depthToColor(norm)
                let p = (y * width + x) * 4
                pixels?[p] = UInt8(r * 255)
                pixels?[p + 1] = UInt8(g * 255)
                pixels?[p + 2] = UInt8(b * 255)
                pixels?[p + 3] = 255
            }
        }
        if let cgImage = context.makeImage() {
            lastDepthFrame = UIImage(cgImage: cgImage)
        }
    }
    private func depthToColor(_ v: Float) -> (Float, Float, Float) {
        if v < 0.33 { let t = v / 0.33; return (t, 0, 0) }
        else if v < 0.66 { let t = (v - 0.33) / 0.33; return (1, t, 0) }
        else { let t = (v - 0.66) / 0.34; return (1, 1, t) }
    }
    
    // MARK: - Stream Send
    private func sendFrames() {
        guard let net = networkManager else { return }
        frameCount += 1
        
        if let cam = lastCameraFrame {
            if let data = resizeImage(cam, maxDimension: 640).jpegData(compressionQuality: 0.4) {
                net.uploadFrame(data) { if case .failure(let e) = $0 { print("❌ Camera err: \(e.localizedDescription)") } }
            }
        }
        // Send LiDAR point cloud JSON instead of image data
        if let arView = arView, let frame = arView.session.currentFrame, let depthMap = frame.sceneDepth?.depthMap {
            let points = extractPointsFromDepth(depthMap, frame: frame)
            if !points.isEmpty {
                net.uploadLidarFrame(points: points) { success in
                    if !success { print("❌ Failed to upload LiDAR JSON points") }
                }
            }
        }


        if frameCount % 10 == 0 { print("📤 Streaming frame #\(frameCount)") }
    }
    
    private func resizeImage(_ img: UIImage, maxDimension: CGFloat) -> UIImage {
        let size = img.size
        let aspect = size.width / size.height
        var new = size
        if size.width > maxDimension || size.height > maxDimension {
            if aspect > 1 { new = CGSize(width: maxDimension, height: maxDimension / aspect) }
            else { new = CGSize(width: maxDimension * aspect, height: maxDimension) }
        }
        if new == size { return img }
        UIGraphicsBeginImageContextWithOptions(new, false, 1.0)
        img.draw(in: CGRect(origin: .zero, size: new))
        let result = UIGraphicsGetImageFromCurrentImageContext()
        UIGraphicsEndImageContext()
        return result ?? img
    }
    
    // MARK: - Point Cloud
    private func extractPointsFromDepth(_ depth: CVPixelBuffer, frame: ARFrame) -> [LidarPoint] {
        var pts: [LidarPoint] = []
        CVPixelBufferLockBaseAddress(depth, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depth, .readOnly) }
        let w = CVPixelBufferGetWidth(depth)
        let h = CVPixelBufferGetHeight(depth)
        guard let base = CVPixelBufferGetBaseAddress(depth) else { return pts }
        let buf = base.assumingMemoryBound(to: Float32.self)
        for y in stride(from: 0, to: h, by: 10) {
            for x in stride(from: 0, to: w, by: 10) {
                let i = y * w + x
                let d = buf[i]
                guard d > 0 && d < 10 else { continue }
                let norm = CGPoint(x: CGFloat(x)/CGFloat(w), y: CGFloat(y)/CGFloat(h))
                let wp = frame.camera.unprojectPoint(norm, ontoPlaneAt: d, orientation: .portrait)
                pts.append(LidarPoint(x: Double(wp.x), y: Double(wp.y), z: Double(wp.z)))
            }
        }
        return pts
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
            ])
    }
}

// MARK: - ARCamera Helper
extension ARCamera {
    func unprojectPoint(_ point: CGPoint, ontoPlaneAt depth: Float, orientation: UIInterfaceOrientation) -> simd_float3 {
        let vMat = self.viewMatrix(for: orientation)
        let pMat = self.projectionMatrix(for: orientation, viewportSize: CGSize(width: 1, height: 1), zNear: 0.001, zFar: 1000)
        let ndc = simd_float4(Float(point.x) * 2 - 1, Float(1 - point.y) * 2 - 1, depth, 1)
        let invP = pMat.inverse, invV = vMat.inverse
        var world = invP * ndc
        world /= world.w
        world = invV * world
        return simd_float3(world.x, world.y, world.z)
    }
}

struct LidarCaptureView_Previews: PreviewProvider {
    static var previews: some View {
        LidarCaptureView().environmentObject(NetworkManager())
    }
}

