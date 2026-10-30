/**
 * arm-bench WebXR Teleoperation Client
 * Based on manipulator_teleop/webxr_api_server implementation
 */

// DOM Elements
const connectBtn = document.getElementById('connect-btn');
const startARBtn = document.getElementById('start-ar-btn');
const statusIndicator = document.getElementById('status-indicator');
const statusText = document.getElementById('status-text');
const connectionCard = document.getElementById('connection-card');
const poseDisplay = document.getElementById('pose-display');
const poseX = document.getElementById('pose-x');
const poseY = document.getElementById('pose-y');
const poseZ = document.getElementById('pose-z');
const gripperBadge = document.getElementById('gripper-badge');
const moveBadge = document.getElementById('move-badge');

// State variables
let socket = null;
let xrSession = null;
let referenceSpace = null;
let animationFrameId = null;
let gripperOpen = false;
let moveEnabled = false;

// Check WebXR support
function checkXRSupport() {
    if ('xr' in navigator) {
        console.log('WebXR is available');
        
        navigator.xr.isSessionSupported('immersive-ar')
            .then((supported) => {
                if (supported) {
                    console.log('WebXR AR supported');
                    startARBtn.disabled = !(socket && socket.readyState === WebSocket.OPEN);
                } else {
                    console.log('WebXR AR not supported on this device');
                    startARBtn.disabled = true;
                    startARBtn.textContent = 'AR Not Supported';
                }
            })
            .catch(err => {
                console.error('Error checking WebXR support:', err);
            });
    } else {
        console.log('WebXR not supported by this browser');
        startARBtn.disabled = true;
        startARBtn.textContent = 'WebXR Not Available';
    }
}

// Connect to WebSocket server
function connectToServer() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const host = window.location.host;
    const wsUrl = `${protocol}//${host}/ws`;
    
    console.log(`Connecting to: ${wsUrl}`);
    statusText.textContent = 'Connecting...';
    
    socket = new WebSocket(wsUrl);
    
    socket.onopen = () => {
        console.log('WebSocket connected');
        statusText.textContent = 'Connected';
        statusIndicator.classList.add('connected');
        connectionCard.classList.add('connected');
        connectBtn.textContent = 'Disconnect';
        connectBtn.classList.remove('btn-primary');
        connectBtn.classList.add('btn-danger');
        
        // Check XR support now that we're connected
        checkXRSupport();
    };
    
    socket.onclose = () => {
        console.log('WebSocket disconnected');
        statusText.textContent = 'Disconnected';
        statusIndicator.classList.remove('connected');
        connectionCard.classList.remove('connected');
        connectBtn.textContent = 'Connect to Server';
        connectBtn.classList.remove('btn-danger');
        connectBtn.classList.add('btn-primary');
        startARBtn.disabled = true;
        
        // Stop XR session if active
        if (xrSession) {
            xrSession.end();
        }
    };
    
    socket.onerror = (error) => {
        console.error('WebSocket error:', error);
        statusText.textContent = 'Connection Error';
    };
    
    socket.onmessage = (event) => {
        // Handle incoming messages (pose broadcasts from other clients)
        try {
            const data = JSON.parse(event.data);
            console.log('Received:', data);
        } catch (e) {
            console.error('Error parsing message:', e);
        }
    };
}

// Disconnect from server
function disconnectFromServer() {
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.close();
    }
}

// Toggle connection
function toggleConnection() {
    if (!socket || socket.readyState === WebSocket.CLOSED || socket.readyState === WebSocket.CLOSING) {
        connectToServer();
    } else {
        disconnectFromServer();
    }
}

// Create AR overlay button
function createARButton(id, text, position) {
    const button = document.createElement('button');
    button.id = id;
    button.innerText = text;
    button.style.cssText = `
        position: absolute;
        bottom: 30px;
        ${position};
        transform: translateX(-50%);
        padding: 16px 24px;
        background: rgba(0, 0, 0, 0.7);
        color: white;
        border: none;
        border-radius: 30px;
        font-size: 16px;
        font-weight: bold;
        box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3);
        pointer-events: auto;
        user-select: none;
        -webkit-user-select: none;
        touch-action: manipulation;
    `;
    
    // Prevent text selection
    button.addEventListener('selectstart', e => e.preventDefault());
    button.addEventListener('dragstart', e => e.preventDefault());
    button.addEventListener('contextmenu', e => e.preventDefault());
    
    return button;
}

// Start WebXR AR session
async function startARSession() {
    try {
        if (!('xr' in navigator)) {
            alert('WebXR not supported');
            return;
        }
        
        console.log('Starting AR session...');
        
        // Create overlay for AR controls
        const overlay = document.createElement('div');
        overlay.id = 'xr-overlay';
        overlay.style.cssText = `
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            z-index: 999999;
            pointer-events: none;
        `;
        document.body.appendChild(overlay);
        
        // Reset control states
        gripperOpen = false;
        moveEnabled = false;
        
        // Create control buttons
        const exitBtn = createARButton('exit-btn', 'Exit', 'left: 50%');
        exitBtn.addEventListener('click', () => {
            if (xrSession) xrSession.end();
        });
        
        const gripperBtn = createARButton('gripper-btn', 'Open Gripper', 'left: 20%');
        gripperBtn.addEventListener('click', () => {
            gripperOpen = !gripperOpen;
            gripperBtn.innerText = gripperOpen ? 'Close Gripper' : 'Open Gripper';
            gripperBtn.style.background = gripperOpen ? 'rgba(46, 204, 113, 0.8)' : 'rgba(0, 0, 0, 0.7)';
            console.log(`Gripper: ${gripperOpen ? 'OPEN' : 'CLOSED'}`);
        });
        
        const moveBtn = createARButton('move-btn', 'Hold to Move', 'left: 80%');
        
        // Momentary move button (active only while pressed)
        const activateMove = () => {
            moveEnabled = true;
            moveBtn.style.background = 'rgba(46, 204, 113, 0.8)';
            console.log('Move: ENABLED');
        };
        
        const deactivateMove = () => {
            moveEnabled = false;
            moveBtn.style.background = 'rgba(0, 0, 0, 0.7)';
            console.log('Move: DISABLED');
        };
        
        moveBtn.addEventListener('mousedown', activateMove);
        moveBtn.addEventListener('touchstart', activateMove);
        moveBtn.addEventListener('mouseup', deactivateMove);
        moveBtn.addEventListener('mouseleave', deactivateMove);
        moveBtn.addEventListener('touchend', deactivateMove);
        moveBtn.addEventListener('touchcancel', deactivateMove);
        
        overlay.appendChild(exitBtn);
        overlay.appendChild(gripperBtn);
        overlay.appendChild(moveBtn);
        
        // Request AR session with DOM overlay
        const sessionOptions = {
            requiredFeatures: ['local'],
            optionalFeatures: ['dom-overlay'],
            domOverlay: { root: overlay }
        };
        
        xrSession = await navigator.xr.requestSession('immersive-ar', sessionOptions);
        console.log('AR session started');
        
        xrSession.addEventListener('end', onSessionEnded);
        
        // Create canvas for WebGL (required even if not rendering)
        const canvas = document.createElement('canvas');
        canvas.style.display = 'none';
        document.body.appendChild(canvas);
        
        const gl = canvas.getContext('webgl', { xrCompatible: true });
        if (!gl) throw new Error('WebGL not supported');
        
        const xrGlLayer = new XRWebGLLayer(xrSession, gl);
        await xrSession.updateRenderState({ baseLayer: xrGlLayer });
        
        // Get reference space
        try {
            referenceSpace = await xrSession.requestReferenceSpace('local-floor');
            console.log('Using local-floor reference space');
        } catch (e) {
            referenceSpace = await xrSession.requestReferenceSpace('local');
            console.log('Using local reference space');
        }
        
        // Start animation loop
        xrSession.requestAnimationFrame(onAnimationFrame);
        
        // Update UI
        startARBtn.disabled = true;
        poseDisplay.classList.add('visible');
        
    } catch (error) {
        console.error('Error starting AR session:', error);
        alert(`Failed to start AR: ${error.message}`);
    }
}

// Animation frame callback
function onAnimationFrame(time, frame) {
    if (!xrSession) return;
    
    animationFrameId = xrSession.requestAnimationFrame(onAnimationFrame);
    
    const pose = frame.getViewerPose(referenceSpace);
    
    if (pose) {
        const position = pose.transform.position;
        const orientation = pose.transform.orientation;
        
        // Update UI
        poseX.textContent = position.x.toFixed(3);
        poseY.textContent = position.y.toFixed(3);
        poseZ.textContent = position.z.toFixed(3);
        
        gripperBadge.textContent = `Gripper: ${gripperOpen ? 'OPEN' : 'CLOSED'}`;
        gripperBadge.classList.toggle('open', gripperOpen);
        
        moveBadge.textContent = `Move: ${moveEnabled ? 'ON' : 'OFF'}`;
        moveBadge.classList.toggle('enabled', moveEnabled);
        
        // Create pose data
        const poseData = {
            timestamp: new Date().toISOString(),
            position: {
                x: position.x.toFixed(4),
                y: position.y.toFixed(4),
                z: position.z.toFixed(4)
            },
            orientation: {
                x: orientation.x.toFixed(4),
                y: orientation.y.toFixed(4),
                z: orientation.z.toFixed(4),
                w: orientation.w.toFixed(4)
            },
            control: {
                gripperOpen: gripperOpen,
                moveEnabled: moveEnabled
            }
        };
        
        // Send to server
        if (socket && socket.readyState === WebSocket.OPEN) {
            socket.send(JSON.stringify(poseData));
        }
    }
}

// Handle session end
function onSessionEnded() {
    console.log('AR session ended');
    
    xrSession = null;
    referenceSpace = null;
    
    // Remove overlay
    const overlay = document.getElementById('xr-overlay');
    if (overlay) overlay.remove();
    
    // Remove hidden canvas
    const canvas = document.querySelector('canvas[style*="display: none"]');
    if (canvas) canvas.remove();
    
    // Update UI
    startARBtn.disabled = false;
    poseDisplay.classList.remove('visible');
}

// Check server status
function checkServerStatus() {
    const url = `${window.location.protocol}//${window.location.host}/server-info`;
    
    fetch(url)
        .then(response => response.json())
        .then(data => {
            console.log('Server status:', data);
        })
        .catch(error => {
            console.error('Server check failed:', error);
        });
}

// Event listeners
connectBtn.addEventListener('click', toggleConnection);
startARBtn.addEventListener('click', startARSession);

// Initialize
document.addEventListener('DOMContentLoaded', () => {
    checkServerStatus();
    checkXRSupport();
});
