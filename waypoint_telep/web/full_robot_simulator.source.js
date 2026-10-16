import * as THREE from '../../../cyclo_intelligence/orchestrator/ui/node_modules/three/build/three.module.js';
import URDFLoader from '../../../cyclo_intelligence/orchestrator/ui/node_modules/urdf-loader/src/URDFLoader.js';
import { OrbitControls } from '../../../cyclo_intelligence/orchestrator/ui/node_modules/three/examples/jsm/controls/OrbitControls.js';
import { TransformControls } from '../../../cyclo_intelligence/orchestrator/ui/node_modules/three/examples/jsm/controls/TransformControls.js';
import { STLLoader } from '../../../cyclo_intelligence/orchestrator/ui/node_modules/three/examples/jsm/loaders/STLLoader.js';

const MIN_MODEL_SIZE = 0.15;

function fallbackMesh(color = 0x888888) {
  return new THREE.Mesh(
    new THREE.BoxGeometry(0.025, 0.025, 0.025),
    new THREE.MeshStandardMaterial({ color, transparent: true, opacity: 0.35 }),
  );
}

function disposeObject(root) {
  root?.traverse((child) => {
    child.geometry?.dispose?.();
    if (Array.isArray(child.material)) child.material.forEach((material) => material.dispose?.());
    else child.material?.dispose?.();
  });
}

function textSprite(text, color = '#ffffff') {
  const canvas = document.createElement('canvas');
  canvas.width = 512;
  canvas.height = 128;
  const context = canvas.getContext('2d');
  context.clearRect(0, 0, canvas.width, canvas.height);
  context.font = '700 52px system-ui, sans-serif';
  context.textAlign = 'center';
  context.textBaseline = 'middle';
  context.lineWidth = 10;
  context.strokeStyle = 'rgba(10, 18, 31, 0.92)';
  context.strokeText(text, canvas.width / 2, canvas.height / 2);
  context.fillStyle = color;
  context.fillText(text, canvas.width / 2, canvas.height / 2);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({
    map: texture,
    transparent: true,
    depthTest: false,
  }));
  sprite.scale.set(0.34, 0.085, 1);
  sprite.renderOrder = 100;
  return sprite;
}

class FullRobotSimulator {
  constructor(canvas) {
    this.canvas = canvas;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x111b2b);
    this.camera = new THREE.PerspectiveCamera(50, 1, 0.01, 100);
    this.renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: true,
      alpha: false,
      powerPreference: 'low-power',
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.1;
    this.controls.minDistance = 0.3;
    this.controls.maxDistance = 10;
    this.robotGroup = new THREE.Group();
    this.robotGroup.rotation.x = -Math.PI / 2;
    this.scene.add(this.robotGroup);
    this.modelGroup = new THREE.Group();
    this.modelGroup.name = 'waypoint_telep_world_to_robot_root';
    this.robotGroup.add(this.modelGroup);
    this.worldAxes = new THREE.AxesHelper(0.5);
    this.worldAxes.name = 'waypoint_telep_fixed_world_axes';
    this.worldAxes.renderOrder = 90;
    this.worldAxes.material.depthTest = false;
    this.robotGroup.add(this.worldAxes);
    this.worldLabel = textSprite('WORLD FIXED');
    this.worldLabel.position.set(0, 0, 0.58);
    this.robotGroup.add(this.worldLabel);
    const axisLabels = [
      ['X', '#ff6673', [0.57, 0, 0]],
      ['Y', '#4fd39e', [0, 0.57, 0]],
      ['Z', '#65a9ff', [0, 0, 0.57]],
    ];
    axisLabels.forEach(([label, color, position]) => {
      const sprite = textSprite(label, color);
      sprite.scale.set(0.11, 0.055, 1);
      sprite.position.set(...position);
      this.robotGroup.add(sprite);
    });
    this.robot = null;
    this.tipLinks = {};
    this.pendingJointValues = {};
    this.trajectoryKeys = {};
    this.pathObjects = {};
    this.waypointObjects = {};
    this.waypointKeys = {};
    this.fitTimer = null;
    this.worldPoseKey = '';
    this.worldFrameName = '';
    this.poseEditorKey = '';
    this.poseEditorMode = 'translate';
    this.poseEditorOnChange = null;
    this.poseEditorTarget = new THREE.Group();
    this.poseEditorTarget.name = 'waypoint_telep_pose_editor_target';
    this.poseEditorVisual = new THREE.Group();
    this.poseEditorVisual.name = 'waypoint_telep_pose_editor_visual';
    this.poseEditorVisual.add(new THREE.AxesHelper(0.16));
    this.poseEditorVisual.add(new THREE.Mesh(
      new THREE.SphereGeometry(0.018, 16, 10),
      new THREE.MeshBasicMaterial({
        color: 0xffd45e,
        transparent: true,
        opacity: 0.86,
        depthTest: false,
      }),
    ));
    this.poseEditorTarget.add(this.poseEditorVisual);
    this.robotGroup.add(this.poseEditorTarget);
    this.poseEditorTarget.visible = false;

    this.transformControls = new TransformControls(this.camera, canvas);
    this.transformControls.setMode(this.poseEditorMode);
    this.transformControls.setSpace('local');
    this.transformControls.setSize(0.72);
    this.transformControls.translationSnap = 0.001;
    this.transformControls.rotationSnap = THREE.MathUtils.degToRad(0.5);
    this.transformControls.attach(this.poseEditorTarget);
    this.transformControls.enabled = false;
    this.transformControlsHelper = this.transformControls.getHelper();
    this.transformControlsHelper.visible = false;
    this.scene.add(this.transformControlsHelper);
    this.transformControls.addEventListener('mouseDown', () => {
      this.controls.enabled = false;
      this.canvas.classList.add('gizmo-dragging');
    });
    this.transformControls.addEventListener('mouseUp', () => {
      this.controls.enabled = true;
      this.canvas.classList.remove('gizmo-dragging');
    });
    this.transformControls.addEventListener('objectChange', () => {
      this.poseEditorKey = '';
      if (!this.poseEditorOnChange) return;
      const quaternion = this.poseEditorMode === 'rotate'
        ? this.poseEditorTarget.quaternion
        : this.poseEditorVisual.quaternion;
      this.poseEditorOnChange({
        position_m: this.poseEditorTarget.position.toArray(),
        orientation_xyzw: quaternion.toArray(),
      });
    });

    this.scene.add(new THREE.HemisphereLight(0xcfe2ff, 0x2a3342, 1.35));
    const keyLight = new THREE.DirectionalLight(0xffffff, 2.2);
    keyLight.position.set(4, 7, 5);
    keyLight.castShadow = true;
    this.scene.add(keyLight);
    const fillLight = new THREE.DirectionalLight(0x8ebcff, 1.0);
    fillLight.position.set(-4, 3, -4);
    this.scene.add(fillLight);
    const grid = new THREE.GridHelper(10, 100, 0x52657d, 0x2a3a50);
    grid.material.transparent = true;
    grid.material.opacity = 0.55;
    this.scene.add(grid);

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(canvas);
    this.resize();
    this.camera.position.set(1.5, 1.3, -1.5);
    this.camera.lookAt(0, 0.6, 0);
    this.controls.target.set(0, 0.6, 0);
    this.renderer.setAnimationLoop(() => {
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    });
  }

  resize() {
    const width = Math.max(1, this.canvas.clientWidth);
    const height = Math.max(1, this.canvas.clientHeight);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }

  async load({ urdfUrl, assetOrigin, tipLinks }) {
    this.tipLinks = { ...tipLinks };
    const loader = new URDFLoader();
    const stlLoader = new STLLoader();
    loader.packages = {
      ffw_description: `${assetOrigin}/urdf/ffw_description`,
      open_manipulator_description: `${assetOrigin}/urdf/open_manipulator_description`,
      realsense2_description: `${assetOrigin}/urdf/realsense2_description`,
    };

    const loadStl = (url, onComplete) => {
      stlLoader.load(
        url,
        (geometry) => {
          geometry.computeVertexNormals();
          onComplete(new THREE.Mesh(
            geometry,
            new THREE.MeshStandardMaterial({
              color: 0xcccccc,
              metalness: 0.25,
              roughness: 0.62,
            }),
          ));
          if (this.robot) this.scheduleCameraFit();
        },
        undefined,
        () => {
          onComplete(fallbackMesh(0xc4474f));
          if (this.robot) this.scheduleCameraFit();
        },
      );
    };

    loader.loadMeshCb = (path, _manager, onComplete) => {
      const swerveMarker = '${swerve_meshes_dir}/';
      const swerveIndex = path.indexOf(swerveMarker);
      if (swerveIndex !== -1) {
        const relativePath = path.slice(swerveIndex + swerveMarker.length);
        loadStl(
          `${loader.packages.ffw_description}/meshes/common/follower/ffw_f_swerve/${relativePath}`,
          onComplete,
        );
        return;
      }
      if (path.startsWith('package://')) {
        const relative = path.slice('package://'.length);
        const slashIndex = relative.indexOf('/');
        const packageName = slashIndex === -1 ? relative : relative.slice(0, slashIndex);
        const relativePath = slashIndex === -1 ? '' : relative.slice(slashIndex + 1);
        const base = loader.packages[packageName];
        if (base && relativePath) loadStl(`${base}/${relativePath}`, onComplete);
        else onComplete(fallbackMesh());
        return;
      }
      const fileIndex = path.indexOf('file:///');
      if (fileIndex !== -1) {
        const absolutePath = path.substring(fileIndex + 'file:///'.length);
        const shareMatch = absolutePath.match(/\/share\/([^/]+)\/(.+)$/);
        if (shareMatch && loader.packages[shareMatch[1]]) {
          loadStl(`${loader.packages[shareMatch[1]]}/${shareMatch[2]}`, onComplete);
        } else {
          onComplete(fallbackMesh());
        }
        return;
      }
      loadStl(path, onComplete);
    };

    const loadedRobot = await new Promise((resolve, reject) => {
      loader.load(urdfUrl, resolve, undefined, reject);
    });
    if (this.robot) {
      this.modelGroup.remove(this.robot);
      disposeObject(this.robot);
    }
    this.waypointObjects = {};
    this.waypointKeys = {};
    this.pathObjects = {};
    this.trajectoryKeys = {};
    this.robot = loadedRobot;
    this.robot.traverse((child) => {
      if (!child.isMesh || !child.material) return;
      const urdfMaterial = child.userData?.urdfMaterial;
      if (urdfMaterial?.color) {
        child.material.color.setRGB(
          urdfMaterial.color.r,
          urdfMaterial.color.g,
          urdfMaterial.color.b,
        );
        if (urdfMaterial.color.a !== undefined && urdfMaterial.color.a < 1) {
          child.material.transparent = true;
          child.material.opacity = urdfMaterial.color.a;
        }
      }
      child.castShadow = true;
      child.receiveShadow = true;
    });
    this.modelGroup.add(this.robot);
    Object.entries(this.tipLinks).forEach(([arm, tipLink]) => {
      const tip = this.robot.links?.[tipLink];
      if (!tip) return;
      const tipAxes = new THREE.AxesHelper(0.13);
      tipAxes.name = `waypoint_telep_${arm}_gripper_axes`;
      tip.add(tipAxes);
    });
    this.setJointValues(this.pendingJointValues);
    this.scheduleCameraFit();
    return this.robot;
  }

  setWorldTransform(frameName, pose) {
    const position = pose?.position_m || [0, 0, 0];
    const orientation = pose?.orientation_xyzw || [0, 0, 0, 1];
    if (position.length !== 3 || orientation.length !== 4) return;
    const key = JSON.stringify([frameName, position, orientation]);
    if (key === this.worldPoseKey) return;
    this.worldPoseKey = key;
    this.modelGroup.position.set(...position.map(Number));
    this.modelGroup.quaternion.set(...orientation.map(Number));
    this.modelGroup.updateMatrixWorld(true);
    if (frameName !== this.worldFrameName) {
      this.worldFrameName = frameName;
      this.robotGroup.remove(this.worldLabel);
      this.worldLabel.material.map?.dispose?.();
      disposeObject(this.worldLabel);
      this.worldLabel = textSprite(`WORLD FIXED (${frameName || 'world'})`);
      this.worldLabel.position.set(0, 0, 0.58);
      this.robotGroup.add(this.worldLabel);
    }
  }

  setJointValues(valuesByName) {
    this.pendingJointValues = { ...valuesByName };
    if (!this.robot) return;
    Object.entries(valuesByName).forEach(([name, value]) => {
      if (this.robot.joints[name] && Number.isFinite(value)) {
        this.robot.joints[name].setJointValue(value);
      }
    });
    this.robot.updateMatrixWorld(true);
  }

  setPoseEditor({ pose, mode = 'translate', visible = true, onChange = null }) {
    this.poseEditorOnChange = onChange;
    this.setPoseEditorMode(mode);
    const position = pose?.position_m || [];
    const orientation = pose?.orientation_xyzw || [];
    const valid = visible
      && position.length === 3
      && orientation.length === 4
      && [...position, ...orientation].every((value) => Number.isFinite(Number(value)));
    this.poseEditorTarget.visible = valid;
    this.transformControls.enabled = valid;
    this.transformControlsHelper.visible = valid;
    if (!valid) return;
    const key = JSON.stringify([mode, position, orientation]);
    if (key === this.poseEditorKey || this.transformControls.dragging) return;
    this.poseEditorKey = key;
    this.poseEditorTarget.position.set(...position.map(Number));
    if (mode === 'rotate') {
      this.poseEditorTarget.quaternion.set(...orientation.map(Number)).normalize();
      this.poseEditorVisual.quaternion.identity();
    } else {
      this.poseEditorTarget.quaternion.identity();
      this.poseEditorVisual.quaternion.set(...orientation.map(Number)).normalize();
    }
    this.poseEditorTarget.updateMatrixWorld(true);
  }

  setPoseEditorMode(mode) {
    const nextMode = mode === 'rotate' ? 'rotate' : 'translate';
    if (nextMode === this.poseEditorMode) return;
    const orientation = (this.poseEditorMode === 'rotate'
      ? this.poseEditorTarget.quaternion
      : this.poseEditorVisual.quaternion).clone();
    this.poseEditorMode = nextMode;
    this.transformControls.setMode(nextMode);
    this.transformControls.setSpace('local');
    if (nextMode === 'rotate') {
      this.poseEditorTarget.quaternion.copy(orientation);
      this.poseEditorVisual.quaternion.identity();
    } else {
      this.poseEditorTarget.quaternion.identity();
      this.poseEditorVisual.quaternion.copy(orientation);
    }
    this.poseEditorKey = '';
    this.poseEditorTarget.updateMatrixWorld(true);
  }

  setTrajectories(trajectories, baseJointValues = {}) {
    Object.entries(trajectories).forEach(([arm, trajectory]) => {
      this.setTrajectory(arm, trajectory, baseJointValues);
    });
  }

  setTrajectory(arm, trajectory, baseJointValues = {}) {
    const finalValues = trajectory?.samples?.at(-1)?.positions_deg || [];
    const key = trajectory
      ? `${trajectory.sample_count}:${trajectory.duration_s}:${finalValues.join(',')}`
      : '';
    if (key === this.trajectoryKeys[arm]) return;
    this.trajectoryKeys[arm] = key;
    if (this.pathObjects[arm]) {
      this.pathObjects[arm].parent?.remove(this.pathObjects[arm]);
      disposeObject(this.pathObjects[arm]);
      this.pathObjects[arm] = null;
    }
    if (!this.robot || !trajectory?.samples?.length) return;
    const tip = this.robot.links?.[this.tipLinks[arm]];
    if (!tip) return;

    const saved = {};
    Object.entries(this.robot.joints).forEach(([name, joint]) => {
      saved[name] = Number(joint.jointValue?.[0] || 0);
    });
    this.setJointValues(baseJointValues);
    const points = [];
    trajectory.samples.forEach((sample) => {
      trajectory.joint_names.forEach((name, index) => {
        const radians = Number(sample.positions_deg[index]) * Math.PI / 180;
        if (this.robot.joints[name] && Number.isFinite(radians)) {
          this.robot.joints[name].setJointValue(radians);
        }
      });
      this.robotGroup.updateMatrixWorld(true);
      const worldPoint = tip.getWorldPosition(new THREE.Vector3());
      points.push(this.modelGroup.worldToLocal(worldPoint));
    });
    this.setJointValues(saved);
    if (points.length >= 2) {
      const geometry = new THREE.BufferGeometry().setFromPoints(points);
      const material = new THREE.LineBasicMaterial({
        color: arm === 'right' ? 0xf16cae : 0x34d5c5,
      });
      this.pathObjects[arm] = new THREE.Line(geometry, material);
      this.modelGroup.add(this.pathObjects[arm]);
    }
  }

  setWaypointSets(sets) {
    Object.entries(sets).forEach(([arm, value]) => {
      this.setWaypoints(arm, value.waypoints, value.baseLink);
    });
  }

  setWaypoints(arm, waypoints, baseLink) {
    const key = JSON.stringify([baseLink, waypoints]);
    if (key === this.waypointKeys[arm]) return;
    this.waypointKeys[arm] = key;
    if (this.waypointObjects[arm]) {
      this.waypointObjects[arm].parent?.remove(this.waypointObjects[arm]);
      disposeObject(this.waypointObjects[arm]);
      this.waypointObjects[arm] = null;
    }
    if (!this.robot || !waypoints?.length) return;
    const base = this.robot.links?.[baseLink];
    if (!base) return;

    const container = new THREE.Group();
    container.name = 'waypoint_telep_target_frames';
    waypoints.forEach((waypoint) => {
      const frame = new THREE.Group();
      const position = waypoint.position_m || [];
      const orientation = waypoint.orientation_xyzw || [];
      if (position.length !== 3 || orientation.length !== 4) return;
      frame.position.set(...position.map(Number));
      frame.quaternion.set(...orientation.map(Number));
      frame.add(new THREE.AxesHelper(0.075));
      frame.add(new THREE.Mesh(
        new THREE.SphereGeometry(0.012, 12, 8),
        new THREE.MeshBasicMaterial({
          color: arm === 'right' ? 0xf16cae : 0xffbd4a,
        }),
      ));
      container.add(frame);
    });
    base.add(container);
    this.waypointObjects[arm] = container;
    this.robot.updateMatrixWorld(true);
  }

  scheduleCameraFit() {
    window.clearInterval(this.fitTimer);
    let attempts = 0;
    let previousSize = 0;
    let stableCount = 0;
    this.fitTimer = window.setInterval(() => {
      attempts += 1;
      this.robotGroup.updateMatrixWorld(true);
      const box = new THREE.Box3().setFromObject(this.robotGroup);
      const size = box.getSize(new THREE.Vector3());
      const maximum = Math.max(size.x, size.y, size.z);
      const change = maximum > 0 ? Math.abs(maximum - previousSize) / maximum : 1;
      stableCount = change < 0.01 && previousSize >= MIN_MODEL_SIZE ? stableCount + 1 : 0;
      previousSize = maximum;
      if (stableCount >= 2 || attempts >= 40) {
        this.fitCamera();
        window.clearInterval(this.fitTimer);
      }
    }, 200);
  }

  fitCamera() {
    if (!this.robot) return;
    this.robotGroup.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(this.robotGroup);
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    center.y = box.min.y + size.y * 0.56;
    const distance = Math.max(Math.max(size.x, size.y, size.z), MIN_MODEL_SIZE) * 1.8;
    this.camera.position.set(
      center.x + distance * 0.7,
      center.y + distance * 0.5,
      center.z - distance * 0.7,
    );
    this.camera.lookAt(center);
    this.controls.target.copy(center);
    this.controls.update();
  }

  resetCamera() {
    this.fitCamera();
  }
}

window.FullRobotSimulator = FullRobotSimulator;
