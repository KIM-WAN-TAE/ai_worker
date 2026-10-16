import { nodeResolve } from '../../cyclo_intelligence/orchestrator/ui/node_modules/@rollup/plugin-node-resolve/dist/es/index.js';

export default {
  input: 'web/full_robot_simulator.source.js',
  output: {
    file: 'web/full_robot_simulator.bundle.js',
    format: 'iife',
    name: 'WaypointFullRobotSimulatorBundle',
    sourcemap: false,
  },
  plugins: [nodeResolve({
    modulePaths: ['../../cyclo_intelligence/orchestrator/ui/node_modules'],
    browser: true,
  })],
};
