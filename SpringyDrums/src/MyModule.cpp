#include "plugin.hpp"
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>

// Update: 09-04-2025 things to change. The Strike Force could have a wider range especially for the soft strike. Check if CPU can be lowered. Flip the way sustain works, left no sustain, right full sustain. Hard limit on the input might be good since some settings exceed the threshold and the force might not be able to control it too well. Also make sure to document as much as possible in the code with comments so it is easily read able.

// Update: 16-06-2025 Cooments from presentation. Noise signal from "wires" needs to be checked. See if it is the noise itself (marked as *noise* as a comment) or the filtering that may cause the repetetive sound. Also some low frequency content from the noise that is not wanted, check bandpass (marked as *bandpass*)



// In terminal go to cd /Users/albinesping/Documents/code/Rack-SDK/Rack/MyPlugin -> make clean -> make -> make install ------- This Will install this code in VCV rack.

// MyModule.svg has been edited using Illustrator, It is important to only have vector files and for this plugin w:150px h:380px, export within this art board. Best way to find knob placement is to use a screenshot of the plugin in vcv rack and have that as a guide.

struct MyModule : Module {
    enum ParamId {
        SIZE_PARAM,
        TENSION_PARAM,
        DAMPING_PARAM,
        STRIKE_POSITION_PARAM,
        SUSTAIN_PARAM,
        SNARE_BLEND_PARAM,
        SNARE_TONE_PARAM,
        STRIKE_FORCE_PARAM,
        PITCH_BEND_PARAM,
        PARAMS_LEN
    };
    
    enum InputId {
        TRIGGER_INPUT,
        INPUTS_LEN
    };
    
    enum OutputId {
        LEFT_OUTPUT,
        RIGHT_OUTPUT,
        OUTPUTS_LEN
    };
    
    enum LightId {
        TRIGGER_LIGHT,
        LIGHTS_LEN
    };
    
    // Configuration
    int sampleRate = 48000;
    float lastInputAmp = 0.f;
    float triggerThreshold = 0.01f;
    int triggerDebounce = 0;
    
    // Grid parameters
    int gridSize = 10;
    std::vector<std::vector<float>> r_mat;
    std::vector<std::vector<float>> theta_mat;
    std::vector<std::vector<float>> u;
    std::vector<std::vector<float>> u_prev;
    std::vector<std::vector<int>> fixedPoints;
    
    // Physical parameters
    float radius;
    float thickness = 0.0002f;
    float sigma = 0.262f;
    float c_original = 0.f;
    
    // Time step and derived parameters
    float dt = 0.f;
    float dr = 0.f;
    float dtheta = 0.f;
    
    // Parameter change tracking
    bool paramChangeRequested = false;
    bool sizeChanged = false;
    bool tensionChanged = false;
    bool snareChanged = false;
    bool strikeForceChanged = false;
    bool pitchBendChanged = false;
    
    // Output buffer
    std::array<std::array<float, 2>, 8> outputBuffer = {}; // increase output buffer, the 8 was 4 before
    
    // Block processing parameters
    int blockSize = 32;
    
    // Snare properties
    std::vector<std::vector<float>> snareResonantHead;
    std::vector<std::vector<float>> snareResonantHeadPrev;
    std::vector<float> wirePositions;
    std::vector<float> wireDisplacements;
    std::vector<float> wireVelocities;
    std::vector<float> wireMasses;
    int numWires = 5; // This part might be unnecessary?
    
    // Noise generation for snare, part of *noise*
    float noiseState = 0.1f;
    float noiseState2 = 0.2f;
    std::array<float, 6> snareFilterStates = {};
    
    // Properties for strike force and pitch bend
    float transientEnvelope = 0.f;
    int strikeTime = 0;
    float originalTension = 0.f;
    
    // NEW: High-pass filter states for snare (added from MATLAB code)
    float HPFilterX1 = 0.f;      // High-pass filter state x(n-1)
    float HPFilterX2 = 0.f;      // High-pass filter state x(n-2)
    float HPFilterY1 = 0.f;      // High-pass filter state y(n-1)
    float HPFilterY2 = 0.f;      // High-pass filter state y(n-2)
    
    MyModule();
    void reset();
    void initializeGrid();
    void updateGrid();
    void strikeDrum(float velocity);
    std::pair<float, float> updatePhysics();
    void process(const ProcessArgs& args) override;
};

MyModule::MyModule() {
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
    
    // Configure parameters
    configParam(SIZE_PARAM, 4.f, 30.f, 12.f, "Drum Size", " inches");
    configParam(TENSION_PARAM, 50.f, 900.f, 200.f, "Tension", " N/m");
    configParam(DAMPING_PARAM, 0.f, 1.f, 0.5f, "Damping");
    configParam(STRIKE_POSITION_PARAM, 0.f, 1.f, 0.8f, "Strike Position");
    configParam(SUSTAIN_PARAM, 0.f, 1.f, 0.5f, "Sustain");
    configParam(SNARE_BLEND_PARAM, 0.f, 1.f, 0.f, "Snare Blend");
    configParam(SNARE_TONE_PARAM, 0.f, 1.f, 0.6f, "Snare Tone");
    configParam(STRIKE_FORCE_PARAM, 0.f, 1.f, 0.5f, "Strike Force");
    configParam(PITCH_BEND_PARAM, 0.f, 1.f, 0.f, "Pitch Bend");
    
    // Configure inputs and outputs
    configInput(TRIGGER_INPUT, "Trigger");
    configOutput(LEFT_OUTPUT, "Left");
    configOutput(RIGHT_OUTPUT, "Right");
    
    // Initialize grids and arrays
    r_mat.resize(gridSize, std::vector<float>(gridSize, 0.f));
    theta_mat.resize(gridSize, std::vector<float>(gridSize, 0.f));
    u.resize(gridSize, std::vector<float>(gridSize, 0.f));
    u_prev.resize(gridSize, std::vector<float>(gridSize, 0.f));
    fixedPoints.resize(gridSize, std::vector<int>(gridSize, 0));
    
    // Initialize snare components
    snareResonantHead.resize(gridSize, std::vector<float>(gridSize, 0.f));
    snareResonantHeadPrev.resize(gridSize, std::vector<float>(gridSize, 0.f));
    
    // Initialize wire parameters
    wirePositions.resize(numWires);
    wireMasses.resize(numWires, 0.00001f);
    wireDisplacements.resize(numWires, 0.f);
    wireVelocities.resize(numWires, 0.f);
    
    for (int i = 0; i < numWires; i++) {
        wirePositions[i] = 0.3f + (0.4f * i / float(numWires - 1));
    }
    
    // Initialize the model
    reset();
}

void MyModule::reset() {
    lastInputAmp = 0.f;
    triggerDebounce = 0;
    
    for (auto& row : outputBuffer) {
        row[0] = 0.f;
        row[1] = 0.f;
    }
    
    // Calculate time step based on sample rate
    dt = 1.f / sampleRate;
    
    // Calculate radius from size
    float size = params[SIZE_PARAM].getValue();
    radius = size * 2.54f / 200.f;  // Convert inches to meters for SI-Units
    
    // Initialize the grid
    initializeGrid();
    
    // Reset parameter change tracking
    paramChangeRequested = false;
    sizeChanged = false;
    tensionChanged = false;
    snareChanged = false;
    strikeForceChanged = false;
    pitchBendChanged = false;
    
    // Reset snare components
    for (auto& row : snareResonantHead) {
        std::fill(row.begin(), row.end(), 0.f);
    }
    for (auto& row : snareResonantHeadPrev) {
        std::fill(row.begin(), row.end(), 0.f);
    }
    
    for (auto& disp : wireDisplacements) {
        disp = 0.f;
    }
    for (auto& vel : wireVelocities) {
        vel = 0.f;
    }
    
    // Reset transient properties
    transientEnvelope = 0.f;
    strikeTime = 0;
    originalTension = params[TENSION_PARAM].getValue();
    
    // Reset noise generator
    noiseState = 0.1f;
    noiseState2 = 0.2f;
    for (auto& state : snareFilterStates) {
        state = 0.f;
    }
    
    // NEW: Reset high-pass filter states
    HPFilterX1 = 0.f;
    HPFilterX2 = 0.f;
    HPFilterY1 = 0.f;
    HPFilterY2 = 0.f;
}

void MyModule::initializeGrid() {
    // Set up the grid with initial positions and fixed boundaries
    
    // Create grid
    dr = radius / (gridSize - 1);
    dtheta = 2.f * M_PI / gridSize;
    
    // Create meshgrid in polar coordinates
    std::vector<float> r_grid(gridSize);
    std::vector<float> theta_grid(gridSize);
    
    for (int i = 0; i < gridSize; i++) {
        r_grid[i] = float(i) * radius / float(gridSize - 1);
        theta_grid[i] = float(i) * 2.f * M_PI / float(gridSize);
    }
    
    // Create polar coordinate matrices
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            r_mat[i][j] = r_grid[i];
            theta_mat[i][j] = theta_grid[j];
        }
    }
    
    // Initialize displacement matrices
    for (auto& row : u) {
        std::fill(row.begin(), row.end(), 0.f);
    }
    for (auto& row : u_prev) {
        std::fill(row.begin(), row.end(), 0.f);
    }
    
    // Initialize fixed boundary points mask
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            // Mark points at the edge as fixed
            if (i == gridSize - 1) {
                fixedPoints[i][j] = 1;
            } else {
                fixedPoints[i][j] = 0;
            }
        }
    }
}

void MyModule::updateGrid() {
    // Updates grid parameters and recalculates derived values
    
    // Update radius and grid spacing
    dr = radius / (gridSize - 1);
    dtheta = 2.f * M_PI / gridSize;
    
    // Update meshgrid for polar coordinates
    std::vector<float> r_grid(gridSize);
    std::vector<float> theta_grid(gridSize);
    
    for (int i = 0; i < gridSize; i++) {
        r_grid[i] = float(i) * radius / float(gridSize - 1);
        theta_grid[i] = float(i) * 2.f * M_PI / float(gridSize);
    }
    
    // Update polar coordinate matrices
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            r_mat[i][j] = r_grid[i];
            theta_mat[i][j] = theta_grid[j];
        }
    }
}

void MyModule::strikeDrum(float velocity) {
    // Store the original tension for pitch bend effects
    originalTension = params[TENSION_PARAM].getValue();
    
    // Scale velocity based on the Strike Force parameter
    // This allows the Strike Force knob to act as a multiplier for the incoming velocity
    float strikeForceParam = params[STRIKE_FORCE_PARAM].getValue();
    
    // Map input velocity (0-1) to a range controlled by the Strike Force knob
    // When Strike Force is at minimum (0), even max velocity will produce minimal strike
    // When Strike Force is at maximum (1), velocity passes through at full scale
    float mappedVelocity = velocity * (0.1f + (strikeForceParam * 0.9f));
    
    // Apply final scaling and limit to reasonable range
    float strikeForce = std::min(1.0f, mappedVelocity) * 0.01f;
    
    // Calculate strike position (radius from center)
    float strikePosition = params[STRIKE_POSITION_PARAM].getValue();
    float strikeRadius = strikePosition * radius;
    
    // Create strike profile using Gaussian function
    float strikeWidth = radius / 6.f; // Narrower strike width for more natural sound
    
    // Apply strike profile to current displacement
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            if (fixedPoints[i][j] == 0) { // Only non-fixed points
                float r = r_mat[i][j];
                
                // Distance from strike radius (circular pattern)
                float distance = std::abs(r - strikeRadius);
                
                // Apply Gaussian profile
                if (distance < strikeWidth * 2.f) {
                    float profile = std::exp(-(distance * distance) / (2.f * strikeWidth * strikeWidth));
                    // Add to current state instead of replacing it
                    u[i][j] = u[i][j] + strikeForce * profile;
                }
            }
        }
    }
    
    // Also reset snare components for a fresh hit, part of the *noise*
    float snareBlend = params[SNARE_BLEND_PARAM].getValue();
    if (snareBlend > 0.f) {
        // Reset wire positions but not velocities
        std::fill(wireDisplacements.begin(), wireDisplacements.end(), 0.f);
        
        // Add a little randomness to velocities
        for (auto& vel : wireVelocities) {
            vel = (random::uniform() - 0.5f) * 0.01f;
        }
    }
    
    // Reset transient envelope and strike time for new hit
    transientEnvelope = 1.0f;
    strikeTime = 0;
}

std::pair<float, float> MyModule::updatePhysics() {
    // Update the physics using a mass-spring system
    
    // Initialize next state
    std::vector<std::vector<float>> u_next(gridSize, std::vector<float>(gridSize, 0.f));
    
    // Calculate mass value
    float mass = sigma * (radius * radius * M_PI) / (gridSize * gridSize);
    
    // Apply pitch bend based on displacement and parameter
    float pitchBendAmount = params[PITCH_BEND_PARAM].getValue();
    float timeFactor = std::exp(-3.0f * strikeTime / (sampleRate * 0.1f)); // Decays over 100ms
    
    // The actual pitch bend factor that will modulate tension
    float bendFactor = 1.0f + (pitchBendAmount * 2.0f * timeFactor);
    
    // Apply to tension and recalculate wave speed
    float effectiveTension = originalTension * bendFactor;
    
    // Calculate spring coefficient from tension and grid spacing
    float k_spring = effectiveTension / (dr * gridSize);
    
    // Calculate damping coefficient
    float damping = params[DAMPING_PARAM].getValue();
    float baseDamping = 0.05f + damping * 0.5f;
    if (damping > 0.8f) {
        float extraDamping = (damping - 0.7f);
        baseDamping = baseDamping + extraDamping;
    }
    
    // Introduce sustain - apply exponential scaling to damping
    float sustain = params[SUSTAIN_PARAM].getValue();
    float sustainFactor = std::exp(-6.f * (1.f - sustain));
    baseDamping = baseDamping * sustainFactor;
    
    // Process each grid point
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            // Skip fixed boundary points
            if (fixedPoints[i][j] == 1) {
                u_next[i][j] = 0.f;
                continue;
            }
            
            // Special case for center point
            if (i == 0) {
                // For center mass, handle all surrounding masses
                float F_y = 0.f;
                for (int j_val = 0; j_val < gridSize; j_val++) {
                    // Calculate spring force from adjacent point
                    float dx = u[i+1][j_val] - u[i][j];
                    
                    // Calculate spring force (Hooke's law: F = -k * dx)
                    float spring_force = k_spring * dx;
                    
                    // Add to force accumulator
                    F_y += spring_force;
                }
                
                // Divide by number of neighbors to normalize
                F_y /= gridSize;
                
                // Calculate acceleration from force (F = ma)
                float accel = F_y / mass;
                
                // Calculate damping force
                float damp_force = -baseDamping * (u[i][j] - u_prev[i][j]) / dt;
                
                // Apply damping
                accel += damp_force / mass;
                
                // Update using Verlet integration
                u_next[i][j] = 2.f * u[i][j] - u_prev[i][j] + accel * dt * dt;
                
                continue;
            }
            
            // Get neighboring points with wrapping for theta
            int i_prev = std::max(0, i-1);
            int i_next = std::min(gridSize-1, i+1);
            int j_next = (j + 1) % gridSize;
            int j_prev = (j - 1 + gridSize) % gridSize;
            
            // Calculate spring forces from radial neighbors
            // Radial springs - scaled by distance from center
            float radial_scale = 1.0f + float(i) / gridSize;
            
            // Force from inner neighbor
            float dx_inner = u[i_prev][j] - u[i][j];
            float F_r_inner = k_spring * dx_inner / radial_scale;
            
            // Force from outer neighbor
            float dx_outer = u[i_next][j] - u[i][j];
            float F_r_outer = k_spring * dx_outer * radial_scale;
            
            // Calculate spring forces from angular neighbors
            // Angular springs - scaled by radius
            float theta_scale = std::max(0.1f, r_mat[i][j]);
            
            // Force from ccw neighbor
            float dx_ccw = u[i][j_prev] - u[i][j];
            float F_theta_ccw = k_spring * dx_ccw * theta_scale;
            
            // Force from cw neighbor
            float dx_cw = u[i][j_next] - u[i][j];
            float F_theta_cw = k_spring * dx_cw * theta_scale;
            
            // Sum all forces
            float F_total = F_r_inner + F_r_outer + F_theta_ccw + F_theta_cw;
            
            // Calculate acceleration
            float accel = F_total / mass;
            
            // Calculate damping force
            float velocity = (u[i][j] - u_prev[i][j]) / dt;
            float damp_force = -baseDamping * velocity;
            
            // Apply damping
            accel += damp_force / mass;
            
            // Update using Verlet integration
            u_next[i][j] = 2.f * u[i][j] - u_prev[i][j] + accel * dt * dt;
        }
    }
    
    // Update states
    // Input scaling to not exceed +- 1 - Fixed bug by applying limits to each element
    float max_input = 1.0f;
    u_prev = u;
    u = u_next;
    
    // Apply hard limiting to each grid point
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            u[i][j] = std::max(-max_input, std::min(max_input, u[i][j]));
        }
    }
    
    // Process snare if enabled
    float snare_output = 0.f;
    float snareBlend = params[SNARE_BLEND_PARAM].getValue();
    
    if (snareBlend > 0.f) {
        // Calculate snare resonant head response
        std::vector<std::vector<float>> snare_u_next(gridSize, std::vector<float>(gridSize, 0.f));
        
        // Get a looser damping for the resonant head
        float snare_damping = baseDamping * 0.7f;
        
        // Update the resonant head - first with coupling from main head
        // UPDATED: Increased coupling factor from 0.3 to 0.4 to match MATLAB code
        float coupling_factor = 0.4f * snareBlend; // Changed from 0.3f to 0.4f
        
        // Simple coupling model
        for (int i = 0; i < gridSize; i++) {
            for (int j = 0; j < gridSize; j++) {
                if (fixedPoints[i][j] == 1) {
                    snare_u_next[i][j] = 0.f;
                    continue;
                }
                
                // Couple with main head (simplified physics), *comment* maybe this can be simplified so make less calculations
                snare_u_next[i][j] = coupling_factor * u[i][j] +
                    (1.0f - snare_damping) * snareResonantHead[i][j] -
                    (1.0f - snare_damping) * snareResonantHeadPrev[i][j];
            }
        }
        
        // Wire physics - calculate wire responses, *comment* maybe this can be simplified so make less calculations
        float wire_response = 0.f;
        float tension_factor = snareBlend;
        
        for (int w = 0; w < numWires; w++) {
            // Get position in grid coordinates
            int wire_pos = round(wirePositions[w] * gridSize);
            
            // Only process valid positions
            if (wire_pos >= 0 && wire_pos < gridSize) {
                // Sample resonant head position at wire contact point
                int head_pos = round(gridSize/2.f); // Center row
                
                // Calculate force on wire (now responds to both positive and negative head movement)
                float head_velocity = (snare_u_next[wire_pos][head_pos] - snareResonantHead[wire_pos][head_pos]) / dt;
                
                // Apply force based on the absolute velocity of the head
                // This makes the snare wires respond in both directions
                float contact_force = std::abs(head_velocity) * tension_factor;
                
                // Basic wire physics
                float wire_mass = wireMasses[w];
                float wire_accel = contact_force / wire_mass - 0.1f * wireVelocities[w];
                
                // Update wire state
                wireVelocities[w] += dt * wire_accel;
                wireDisplacements[w] += dt * wireVelocities[w];
                
                // Bidirectional response: Check for wire hitting the head in both directions
                if (std::abs(wireDisplacements[w]) > 0.002f) {  // Threshold for contact
                    // Add to response proportional to displacement amplitude
                    wire_response += std::abs(wireDisplacements[w]) * 0.8f;
                    
                    // Wire bounces off with some energy loss, in the opposite direction
                    wireVelocities[w] = -wireVelocities[w] * 0.85f;
                    
                    // Reduce displacement but maintain direction
                    wireDisplacements[w] = 0.001f * (wireDisplacements[w] > 0 ? 1.0f : -1.0f);
                }
            }
        }
        
        // Update the resonant head state
        snareResonantHeadPrev = snareResonantHead;
        snareResonantHead = snare_u_next;
        
        // Calculate total membrane energy for envelope
        float membrane_energy = 0.f;
        for (const auto& row : snare_u_next) {
            for (const auto& val : row) {
                membrane_energy += std::abs(val);
            }
        }
        
        // Generate real-time noise component *noise* here the noise is made
        noiseState = 3.99f * noiseState * (1.f - noiseState);
        noiseState2 = 3.99f * noiseState2 * (1.f - noiseState2);
        
        // Combine the two noise sources
        float noise_sample = (noiseState - 0.5f) + (noiseState2 - 0.5f) * 0.5f;
        
        // Apply filtering based on SnareTone parameter - UPDATED based on MATLAB code
        float snareTone = params[SNARE_TONE_PARAM].getValue();
        
        // 1. Apply first-order low pass filter with higher cutoff frequencies
        // Use an exponential mapping to prevent darkening at higher values
        float cutoffLow = 0.15f + std::exp(snareTone * 1.2f) / std::exp(1.2f) * 0.85f;
        
        // 2. Apply first-order high pass filter with more aggressive curve
        float cutoffHigh = 0.1f + std::exp(snareTone * 1.5f) / std::exp(1.5f) * 0.85f;
        
        snareFilterStates[0] = snareFilterStates[0] +
            cutoffLow * (noise_sample - snareFilterStates[0]);
        
        snareFilterStates[1] = cutoffHigh *
            (snareFilterStates[1] + noise_sample - snareFilterStates[2]);
        snareFilterStates[2] = noise_sample;
        
        // 3. Blend the two filtered signals with high frequency emphasis
        float highFreqEmphasis = 1.0f + snareTone * 0.5f;
        float filtered_noise = snareFilterStates[0] + (snareFilterStates[1] * highFreqEmphasis);
        
        // 4. Apply less resonance at higher tone settings to prevent darkening
        float resonance = 0.7f - snareTone * 0.4f;  // Higher tone = LESS resonance
        snareFilterStates[3] = snareFilterStates[3] * resonance +
            filtered_noise * (1.f - resonance);
        
        // Apply limiting to prevent clipping
        snareFilterStates[3] = std::tanh(snareFilterStates[3]);
        
        // Apply envelope to noise based on membrane energy
        float noise_envelope = 5.f * membrane_energy;
        float noise_component = snareFilterStates[3] * noise_envelope;
        
        // NEW: Apply high-pass filtering to the snare (both wire and noise components)
        // Using a biquad filter for 200Hz high-pass (pre-calculated 2nd order Butterworth)
        float b0 = 0.9833f;
        float b1 = -1.9665f;
        float b2 = 0.9833f;
        float a1 = -1.9663f;
        float a2 = 0.9666f;
        
        // Apply high-pass filter to combined snare sound
        float snare_combined = (wire_response * 0.4f) + noise_component;
        float hp_y0 = b0 * snare_combined + b1 * HPFilterX1 + b2 * HPFilterX2 -
                       a1 * HPFilterY1 - a2 * HPFilterY2;
                    
        // Update filter states
        HPFilterX2 = HPFilterX1;
        HPFilterX1 = snare_combined;
        HPFilterY2 = HPFilterY1;
        HPFilterY1 = hp_y0;
        
        // Boost filtered output to compensate for low-end loss
        snare_output = hp_y0 * 1.5f;
        
        // Apply limiting to prevent clipping
        snare_output = std::tanh(snare_output * 0.8f) * 0.15f;
    }
    
    // Find radius indexes for inner and outer regions
    int innerIdx = std::max(0, int(round(gridSize * 0.5f)));
    int outerIdx = std::min(gridSize-1, int(round(gridSize * 0.5f)));
    
    // Sample from these regions
    float leftSample = 0.f;
    float rightSample = 0.f;
    
    // Inner region (left channel)
    float innerSum = 0.f;
    int innerCount = 0;
    for (int i = 0; i <= innerIdx; i++) {
        for (int j = 0; j < gridSize; j++) {
            innerSum += u[i][j];
            innerCount++;
        }
    }
    leftSample = innerCount > 0 ? innerSum / innerCount : 0.f;
    
    // Outer region (right channel)
    float outerSum = 0.f;
    int outerCount = 0;
    for (int i = innerIdx; i <= outerIdx; i++) {
        for (int j = 0; j < gridSize; j++) {
            outerSum += u[i][j];
            outerCount++;
        }
    }
    rightSample = outerCount > 0 ? outerSum / outerCount : 0.f;
    
    // Apply transient envelope to enhance strike force
    float strikeForce = params[STRIKE_FORCE_PARAM].getValue();
    float transientBoost = 1.0f + (transientEnvelope * strikeForce * 3.0f);
    leftSample *= transientBoost;
    rightSample *= transientBoost;
    
    // Mix in snare sound if enabled
    if (snareBlend > 0.f) {
        float snare_mix = snareBlend;
        float drum_mix = 1.0f - snare_mix;
        
        leftSample = (drum_mix * leftSample) + (snare_mix * snare_output);
        rightSample = (drum_mix * rightSample) + (snare_mix * snare_output);
    }
    
    // NEW: Force mono output - both channels get the same signal
    float monoSample = (leftSample + rightSample) * 0.5f;
    leftSample = monoSample;
    rightSample = monoSample;
    
    // Apply smoothing using output buffer
    for (size_t i = 0; i < outputBuffer.size() - 1; i++) {
        outputBuffer[i] = outputBuffer[i + 1];
    }
    outputBuffer[outputBuffer.size() - 1][0] = leftSample;
    outputBuffer[outputBuffer.size() - 1][1] = rightSample;
    
    float leftOut = 0.f;
    float rightOut = 0.f;
    for (const auto& sample : outputBuffer) {
        leftOut += sample[0];
        rightOut += sample[1];
    }
    leftOut /= outputBuffer.size();
    rightOut /= outputBuffer.size();
    
    // Apply hard limiting to the output
    float max_amp = 1.0f;
    leftOut = std::max(-max_amp, std::min(leftOut, max_amp));
    rightOut = std::max(-max_amp, std::min(rightOut, max_amp));
    
    return {leftOut, rightOut};
}

void MyModule::process(const ProcessArgs& args) {
    // Get sample rate if it has changed
    if (sampleRate != args.sampleRate) {
        sampleRate = args.sampleRate;
        dt = 1.f / sampleRate;
        reset();
    }
    
    // Check if parameters have changed
    float currentSize = params[SIZE_PARAM].getValue();
    float currentTension = params[TENSION_PARAM].getValue();
    float currentSnareBlend = params[SNARE_BLEND_PARAM].getValue();
    float currentStrikeForce = params[STRIKE_FORCE_PARAM].getValue();
    float currentPitchBend = params[PITCH_BEND_PARAM].getValue();
    
    // Detect parameter changes
    static float lastSize = currentSize;
    static float lastTension = currentTension;
    static float lastSnareBlend = currentSnareBlend;
    static float lastStrikeForce = currentStrikeForce;
    static float lastPitchBend = currentPitchBend;
    
    if (currentSize != lastSize) {
        sizeChanged = true;
        paramChangeRequested = true;
        lastSize = currentSize;
    }
    
    if (currentTension != lastTension) {
        tensionChanged = true;
        paramChangeRequested = true;
        lastTension = currentTension;
    }
    
    if (currentSnareBlend != lastSnareBlend) {
        snareChanged = true;
        paramChangeRequested = true;
        lastSnareBlend = currentSnareBlend;
    }
    
    if (currentStrikeForce != lastStrikeForce) {
        strikeForceChanged = true;
        paramChangeRequested = true;
        lastStrikeForce = currentStrikeForce;
    }
    
    if (currentPitchBend != lastPitchBend) {
        pitchBendChanged = true;
        paramChangeRequested = true;
        lastPitchBend = currentPitchBend;
    }
    
    // Check if parameters have changed and need updating
    if (paramChangeRequested) {
        // Handle parameter changes safely
        if (tensionChanged) {
            tensionChanged = false;
        }
        
        if (sizeChanged) {
            radius = currentSize * 2.54f / 200.f;
            updateGrid();
            sizeChanged = false;
        }
        
        if (snareChanged) {
            snareChanged = false;
        }
        
        if (strikeForceChanged) {
            strikeForceChanged = false;
        }
        
        if (pitchBendChanged) {
            pitchBendChanged = false;
        }
        
        paramChangeRequested = false;
    }
    
    // Get input signal
    float input = inputs[TRIGGER_INPUT].getVoltage() / 10.f; // Scale from -10V..10V to -1..1
    
    // Check for trigger (drum hit)
    if (std::abs(input) > triggerThreshold &&
        std::abs(input) > lastInputAmp * 1.2f &&
        triggerDebounce == 0) {
        
        // Strike the drum with velocity proportional to input amplitude
        strikeDrum(std::min(1.0f, std::abs(input) * 3.0f));
        
        // Set debounce counter
        triggerDebounce = round(sampleRate * 0.02f);  // 20ms debounce
        
        // Light the trigger LED
        lights[TRIGGER_LIGHT].setBrightness(1.0f);
    } else {
        // Fade the trigger light
        lights[TRIGGER_LIGHT].setBrightness(std::max(0.0f, lights[TRIGGER_LIGHT].getBrightness() - 0.05f));
    }
    
    // Update debounce counter
    if (triggerDebounce > 0) {
        triggerDebounce--;
    }
    
    lastInputAmp = std::abs(input);
    
    // Update strike time counter for transient envelope
    strikeTime++;
    
    // Update transient envelope - quick attack, slower decay
    float attackTime = 0.005f * sampleRate; // 5ms attack
    float decayTime = 0.1f * sampleRate; // 100ms decay
    
    if (strikeTime < attackTime) {
        // Attack phase - fast rise
        transientEnvelope = std::min(1.0f, strikeTime / attackTime);
    } else {
        // Decay phase - exponential decay
        float decayFactor = std::exp(-3.0f * (strikeTime - attackTime) / decayTime);
        transientEnvelope = std::max(0.0f, decayFactor);
    }
    
    // Update physics and get audio output
    std::pair<float, float> audioOutputs = updatePhysics();
    float leftOut = audioOutputs.first;
    float rightOut = audioOutputs.second;
    
    // Output the audio
    outputs[LEFT_OUTPUT].setVoltage(leftOut * 5.0f);  // Scale to roughly -5V..5V
    outputs[RIGHT_OUTPUT].setVoltage(rightOut * 5.0f);
}

// Define the ModuleWidget for the UI
struct MyModuleWidget : ModuleWidget {
    MyModuleWidget(MyModule* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/MyModule.svg")));

        // Add the screws
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // Add the knobs - Left column
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(16.93, 19.30)), module, MyModule::SIZE_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(16.93, 37.93)), module, MyModule::TENSION_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(16.93, 56.56)), module, MyModule::DAMPING_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(16.93, 75.18)), module, MyModule::STRIKE_POSITION_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(16.93, 93.81)), module, MyModule::SUSTAIN_PARAM));

        // Add the knobs - Right column
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(33.87, 19.30)), module, MyModule::SNARE_BLEND_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(33.87, 37.93)), module, MyModule::SNARE_TONE_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(33.87, 56.56)), module, MyModule::STRIKE_FORCE_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(33.87, 75.18)), module, MyModule::PITCH_BEND_PARAM));

        // Add the input/output ports
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(12.53, 113.45)), module, MyModule::TRIGGER_INPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(25.06, 113.45)), module, MyModule::LEFT_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(37.59, 113.45)), module, MyModule::RIGHT_OUTPUT));

        // Add the lights
        addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(12.53, 121.07)), module, MyModule::TRIGGER_LIGHT));
    }
};

// Create the model - this registers our module with the VCV Rack plugin
Model* modelMyModule = createModel<MyModule, MyModuleWidget>("MyModule");

