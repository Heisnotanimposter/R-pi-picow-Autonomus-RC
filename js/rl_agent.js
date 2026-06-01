/**
 * RL (Reinforcement Learning) Agent Module
 * Implements a tabular Q-Learning agent with discretized state space
 * specifically tuned for real-time in-browser training.
 */

class RLAgent {
  constructor() {
    this.learningRate = 0.2;
    this.discountFactor = 0.9;
    this.epsilon = 0.2; // default exploration rate
    
    // Discrete action values (steering angles in radians, positive = Left)
    this.actions = [0.4, 0.18, 0.0, -0.18, -0.4]; // Hard L, Soft L, Straight, Soft R, Hard R
    
    // Q-Table map: "cteIndex_yawIndex_obsIndex" -> array of 5 action values
    this.qTable = {};
    
    // Reward weights
    this.wSpeed = 1.0;
    this.wCte = 1.5;
    this.wCollision = 15.0;

    // Neural Network weights simulation for live node graph visualization
    this.nnWeights = this.initializeMockNN();
    this.nnActivations = {
      inputs: [0, 0, 0, 0, 0],
      hidden: [0, 0, 0, 0],
      outputs: [0, 0, 0, 0, 0]
    };

    // Metrics
    this.episodeCount = 0;
    this.stepCount = 0;
    this.episodeReward = 0.0;
    this.rewardHistory = [];
    this.maxHistoryLength = 50;

    this.lastStateKey = null;
    this.lastActionIdx = null;
  }

  initializeMockNN() {
    // 5 inputs -> 4 hidden -> 5 outputs weights
    const inputToHidden = [];
    for (let i = 0; i < 5; i++) {
      const row = [];
      for (let j = 0; j < 4; j++) {
        row.push((Math.random() - 0.5) * 2.0);
      }
      inputToHidden.push(row);
    }
    const hiddenToOutput = [];
    for (let i = 0; i < 4; i++) {
      const row = [];
      for (let j = 0; j < 5; j++) {
        row.push((Math.random() - 0.5) * 2.0);
      }
      hiddenToOutput.push(row);
    }
    return { inputToHidden, hiddenToOutput };
  }

  // Helper to discretize state space
  discretizeState(cte, yawError, rawDistanceCm) {
    // 1. Cross-track error (7 buckets: extremely left to extremely right)
    let cteIdx = 3; // centered
    if (cte < -4.0) cteIdx = 0;
    else if (cte < -1.5) cteIdx = 1;
    else if (cte < -0.4) cteIdx = 2;
    else if (cte > 4.0) cteIdx = 6;
    else if (cte > 1.5) cteIdx = 5;
    else if (cte > 0.4) cteIdx = 4;

    // 2. Heading error (5 buckets: far left to far right)
    let yawIdx = 2; // straight
    if (yawError < -0.6) yawIdx = 0;
    else if (yawError < -0.15) yawIdx = 1;
    else if (yawError > 0.6) yawIdx = 4;
    else if (yawError > 0.15) yawIdx = 3;

    // 3. Obstacle Ahead range (2 buckets: clear or obstacle ahead within 150cm)
    const obsIdx = (rawDistanceCm < 150) ? 1 : 0;

    return `${cteIdx}_${yawIdx}_${obsIdx}`;
  }

  getQValues(stateKey) {
    if (!this.qTable[stateKey]) {
      // Initialize with small random values to encourage initial exploration
      this.qTable[stateKey] = Array(this.actions.length).fill(0).map(() => (Math.random() - 0.5) * 0.1);
    }
    return this.qTable[stateKey];
  }

  // Act selection using Epsilon-Greedy policy
  selectAction(cte, yawError, rawDistanceCm) {
    const stateKey = this.discretizeState(cte, yawError, rawDistanceCm);
    const qValues = this.getQValues(stateKey);
    
    let actionIdx;
    if (Math.random() < this.epsilon) {
      // Explore: random action
      actionIdx = Math.floor(Math.random() * this.actions.length);
    } else {
      // Exploit: best action
      let maxQ = -999999;
      let bestIndices = [];
      for (let i = 0; i < qValues.length; i++) {
        if (qValues[i] > maxQ) {
          maxQ = qValues[i];
          bestIndices = [i];
        } else if (qValues[i] === maxQ) {
          bestIndices.push(i);
        }
      }
      actionIdx = bestIndices[Math.floor(Math.random() * bestIndices.length)];
    }

    // Cache values for learning update in next step
    this.lastStateKey = stateKey;
    this.lastActionIdx = actionIdx;

    // Feed forward NN simulator for visualization
    this.updateNNVisualizer(cte, yawError, rawDistanceCm, qValues, actionIdx);

    return this.actions[actionIdx];
  }

  // Run the Q-learning Temporal Difference update rule
  updateQTable(cte, yawError, rawDistanceCm, reward, done) {
    if (this.lastStateKey === null || this.lastActionIdx === null) return;

    const nextStateKey = this.discretizeState(cte, yawError, rawDistanceCm);
    const qValues = this.getQValues(this.lastStateKey);
    const nextQValues = this.getQValues(nextStateKey);

    // Max Q value for next state
    const maxNextQ = done ? 0.0 : Math.max(...nextQValues);

    // Temporal Difference (TD) target
    const tdTarget = reward + this.discountFactor * maxNextQ;
    
    // TD update formula
    qValues[this.lastActionIdx] += this.learningRate * (tdTarget - qValues[this.lastActionIdx]);

    this.episodeReward += reward;
    this.stepCount++;
  }

  // Reset episode metrics
  endEpisode() {
    this.rewardHistory.push(this.episodeReward);
    if (this.rewardHistory.length > this.maxHistoryLength) {
      this.rewardHistory.shift();
    }
    
    this.episodeCount++;
    const finalReward = this.episodeReward;
    this.episodeReward = 0.0;
    this.stepCount = 0;
    this.lastStateKey = null;
    this.lastActionIdx = null;

    return finalReward;
  }

  resetAgent() {
    this.qTable = {};
    this.rewardHistory = [];
    this.episodeCount = 0;
    this.episodeReward = 0.0;
    this.stepCount = 0;
    this.lastStateKey = null;
    this.lastActionIdx = null;
  }

  // Update neural network activations based on state and Q-values for GUI visualizer
  updateNNVisualizer(cte, yawError, rawDistanceCm, qValues, selectedIdx) {
    // 1. Set inputs (discretized values normalized from -1.0 to 1.0)
    this.nnActivations.inputs[0] = Math.max(-1.0, Math.min(1.0, cte / 4.0));
    this.nnActivations.inputs[1] = Math.max(-1.0, Math.min(1.0, yawError / 1.0));
    this.nnActivations.inputs[2] = rawDistanceCm < 150 ? 1.0 : -1.0;
    this.nnActivations.inputs[3] = Math.max(-1.0, Math.min(1.0, (cte - 0) * 0.5)); // rate of error change
    this.nnActivations.inputs[4] = Math.random() * 0.2 - 0.1; // small bias

    // 2. Feedforward pass to hidden layer
    for (let j = 0; j < 4; j++) {
      let sum = 0;
      for (let i = 0; i < 5; i++) {
        sum += this.nnActivations.inputs[i] * this.nnWeights.inputToHidden[i][j];
      }
      this.nnActivations.hidden[j] = Math.tanh(sum); // Activation function
    }

    // 3. Project output Q-values directly
    for (let k = 0; k < 5; k++) {
      // Softmax/scaling representation of output activations
      this.nnActivations.outputs[k] = qValues[k];
    }
    
    // Rotate weights slightly during training to show network "learning"/evolving
    for (let i = 0; i < 5; i++) {
      for (let j = 0; j < 4; j++) {
        this.nnWeights.inputToHidden[i][j] += (Math.random() - 0.5) * 0.02;
        // Keep within bounds
        this.nnWeights.inputToHidden[i][j] = Math.max(-2.0, Math.min(2.0, this.nnWeights.inputToHidden[i][j]));
      }
    }
  }
}

// Export for browser
window.RLAgent = RLAgent;
