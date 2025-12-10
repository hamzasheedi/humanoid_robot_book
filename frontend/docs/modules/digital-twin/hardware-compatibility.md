---
title: Digital Twin Hardware Compatibility Guidelines
---

# Digital Twin Hardware Compatibility Guidelines

This guide covers hardware compatibility considerations for digital twin implementations using Gazebo and Unity. It addresses the three-tier hardware approach (On-Premise Lab, Ether Lab Cloud, and Economy Jetson Student Kit) as specified in the research decisions.

## Overview

Digital twin simulations require significant computational resources, especially for realistic physics and rendering. Different hardware configurations provide different capabilities and limitations that must be considered when developing and deploying simulations.

## Hardware Tier Compatibility

### On-Premise Lab Configuration
**Specifications:**
- **OS**: Ubuntu 22.04 LTS, Windows 10/11
- **CPU**: 8+ cores, 3.0GHz+ recommended
- **RAM**: 16GB+ (32GB recommended)
- **GPU**: Dedicated graphics card (e.g., RTX 3070 or equivalent)
- **Storage**: SSD with 50GB+ free space

**Compatibility:**
- ✅ Gazebo: Full compatibility with high-fidelity physics
- ✅ Unity: Full compatibility with high-quality rendering
- ✅ NVIDIA Isaac: Full compatibility with Isaac Sim
- ✅ Multi-robot simulation: Up to 10+ robots possible
- ⚠️ Consider headless mode for heavy simulations

### Ether Lab (Cloud) Configuration
**Specifications:**
- **Environment**: Cloud-based computing resources
- **GPU**: Virtualized GPU resources (e.g., NVv4 with M60, or V100)
- **OS**: Ubuntu 22.04 LTS in container/VM
- **Network**: Stable internet connection required
- **Rendering**: Headless mode recommended

**Compatibility:**
- ✅ Gazebo: Full compatibility in headless mode
- ⚠️ Unity: Limited compatibility (headless simulation only)
- ✅ NVIDIA Isaac: Compatible via Isaac Sim cloud containers
- ⚠️ Rendering: Limited to non-visual sensors
- ⚠️ Latency: Network-dependent performance

### Economy Jetson Student Kit Configuration
**Specifications:**
- **Hardware**: NVIDIA Jetson Nano, Xavier NX, or AGX Orin
- **OS**: JetPack SDK (Ubuntu-based)
- **CPU**: ARM-based multi-core processor
- **GPU**: Integrated NVIDIA GPU (limited compute capability)
- **RAM**: 4GB (Nano) to 32GB (AGX Orin)
- **Storage**: microSD card or NVMe SSD

**Compatibility:**
- ⚠️ Gazebo: Limited compatibility (simple models and scenes only)
- ❌ Unity: Not recommended due to resource constraints
- ✅ NVIDIA Isaac: Compatible with Isaac ROS packages on Jetson
- ⚠️ Physics complexity: Simplified models required
- ⚠️ Simulation speed: May be significantly reduced

## Simulation Optimization Strategies

### For Resource-Constrained Systems
1. **Simplified Models**: Use low-polygon meshes and simplified collision geometries
2. **Reduced Physics Complexity**: Use fewer joints and simpler dynamics
3. **Lower Update Rates**: Reduce simulation and control frequencies
4. **Headless Operation**: Disable graphics rendering when possible
5. **Smaller Worlds**: Limit environment size and complexity

### For High-Performance Systems
1. **Realistic Models**: Use high-fidelity meshes and accurate physics properties
2. **Complex Environments**: Include detailed scenes with multiple objects
3. **Advanced Sensors**: Implement realistic sensor models (LiDAR, cameras, IMU)
4. **Multi-Robot Systems**: Simulate multiple robots with complex interactions
5. **Advanced Physics**: Use complex contact models and realistic material properties

## Platform-Specific Guidelines

### Gazebo Optimization
- **Physics Engine**: Use DART for better stability or ODE for performance
- **Visual Quality**: Adjust quality settings based on system capabilities
- **Sensors**: Use the `disable_rendering` option for headless operation
- **Real-time Factor**: Monitor and adjust to maintain real-time performance

### Unity Optimization
- **Graphics Settings**: Reduce quality settings for less powerful systems
- **LOD System**: Implement Level of Detail to reduce complexity at distance
- **Occlusion Culling**: Use for complex scenes to reduce rendering load
- **Lighting**: Use baked lighting to reduce runtime computation

## Performance Monitoring

Monitor these key metrics:
- **CPU Usage**: Keep below 80% sustained
- **GPU Memory**: Monitor VRAM usage to avoid overallocation
- **Simulation Speed**: Compare actual vs. real-time performance
- **Battery Life**: For mobile Jetson platforms, monitor power consumption

## Troubleshooting Common Issues

### Low Performance
- **Gazebo**: Reduce physics engine update rate, disable rendering, simplify models
- **Unity**: Reduce rendering quality, use less complex meshes, disable unnecessary effects
- **Jetson**: Switch to headless operation, reduce simulation complexity

### Stability Issues
- **Physics**: Increase solver iterations, reduce time step, check mass properties
- **Network**: For cloud systems, ensure stable connection, consider local caching

### Compatibility Problems
- **Drivers**: Ensure proper GPU drivers are installed
- **Dependencies**: Verify all required packages and libraries are available
- **ROS Distribution**: Confirm compatibility with ROS 2 Humble Hawksbill

## Testing and Validation

Always test your simulation with the target hardware configuration:
1. Execute the planned simulation scenarios
2. Monitor system resource usage
3. Verify that simulation behavior matches expectations
4. Test the entire pipeline including ROS 2 integration

## Best Practices

- Profile your simulation on each target platform
- Provide multiple complexity levels of the same model
- Document performance requirements for each simulation scenario
- Include fallback configurations for resource-constrained systems
- Consider hybrid approaches (cloud simulation with local control)