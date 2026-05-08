import { Suspense, useEffect, useState } from "react";
import { Canvas, useLoader, useFrame } from "@react-three/fiber";
import { OrbitControls, Html } from "@react-three/drei";
import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";

// ---------------------------------------------------------------
// MANTA — corrected architecture viewer
//
// The pilot is the fuselage. A spine yoke + arm-aligned LE spar +
// body-mounted TE spar + telescoping wrist/ankle tip extensions
// form the rigid wing structure, deploying as the pilot spreads
// from a wingsuit-tucked posture.
//
// We render two CAD keyframes (stowed at deploy=0, deployed at
// deploy=1) generated from cad/build.py. Slider crossfades between
// them; "Play" auto-ramps in 2.0 s.
// ---------------------------------------------------------------

type Keyframe = {
  url: string;
  color: string;
  opacity: number;
};

const STOWED: Keyframe[] = [
  { url: "/models/stowed/torso.stl", color: "#a87d52", opacity: 0.9 },
  { url: "/models/stowed/head.stl", color: "#c4956a", opacity: 0.95 },
  { url: "/models/stowed/spine_yoke.stl", color: "#1a1a1a", opacity: 0.95 },
  { url: "/models/stowed/upper_arm_right.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/stowed/forearm_right.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/stowed/hand_right.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/stowed/upper_arm_left.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/stowed/forearm_left.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/stowed/hand_left.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/stowed/le_spar_right.stl", color: "#222", opacity: 0.95 },
  { url: "/models/stowed/le_spar_left.stl", color: "#222", opacity: 0.95 },
  { url: "/models/stowed/upper_leg_right.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/stowed/lower_leg_right.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/stowed/upper_leg_left.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/stowed/lower_leg_left.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/stowed/te_spar_right_stage1.stl", color: "#444", opacity: 0.95 },
  { url: "/models/stowed/te_spar_right_stage2.stl", color: "#444", opacity: 0.95 },
  { url: "/models/stowed/te_spar_right_stage3.stl", color: "#444", opacity: 0.95 },
  { url: "/models/stowed/te_spar_left_stage1.stl", color: "#444", opacity: 0.95 },
  { url: "/models/stowed/te_spar_left_stage2.stl", color: "#444", opacity: 0.95 },
  { url: "/models/stowed/te_spar_left_stage3.stl", color: "#444", opacity: 0.95 },
  { url: "/models/stowed/wrist_ext_right_stage1.stl", color: "#222", opacity: 0.95 },
  { url: "/models/stowed/wrist_ext_right_stage2.stl", color: "#222", opacity: 0.95 },
  { url: "/models/stowed/wrist_ext_right_stage3.stl", color: "#222", opacity: 0.95 },
  { url: "/models/stowed/wrist_ext_left_stage1.stl", color: "#222", opacity: 0.95 },
  { url: "/models/stowed/wrist_ext_left_stage2.stl", color: "#222", opacity: 0.95 },
  { url: "/models/stowed/wrist_ext_left_stage3.stl", color: "#222", opacity: 0.95 },
];

const DEPLOYED: Keyframe[] = [
  { url: "/models/deployed/torso.stl", color: "#a87d52", opacity: 0.9 },
  { url: "/models/deployed/head.stl", color: "#c4956a", opacity: 0.95 },
  { url: "/models/deployed/spine_yoke.stl", color: "#1a1a1a", opacity: 0.95 },
  { url: "/models/deployed/upper_arm_right.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/deployed/forearm_right.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/deployed/hand_right.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/deployed/upper_arm_left.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/deployed/forearm_left.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/deployed/hand_left.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/deployed/le_spar_right.stl", color: "#222", opacity: 0.95 },
  { url: "/models/deployed/le_spar_left.stl", color: "#222", opacity: 0.95 },
  { url: "/models/deployed/upper_leg_right.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/deployed/lower_leg_right.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/deployed/upper_leg_left.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/deployed/lower_leg_left.stl", color: "#a87d52", opacity: 0.85 },
  { url: "/models/deployed/le_spar_right.stl", color: "#222", opacity: 0.95 },
  { url: "/models/deployed/le_spar_left.stl", color: "#222", opacity: 0.95 },
  { url: "/models/deployed/te_spar_right_stage1.stl", color: "#444", opacity: 0.95 },
  { url: "/models/deployed/te_spar_right_stage2.stl", color: "#444", opacity: 0.95 },
  { url: "/models/deployed/te_spar_right_stage3.stl", color: "#444", opacity: 0.95 },
  { url: "/models/deployed/te_spar_left_stage1.stl", color: "#444", opacity: 0.95 },
  { url: "/models/deployed/te_spar_left_stage2.stl", color: "#444", opacity: 0.95 },
  { url: "/models/deployed/te_spar_left_stage3.stl", color: "#444", opacity: 0.95 },
  { url: "/models/deployed/wrist_ext_right_stage1.stl", color: "#222", opacity: 0.95 },
  { url: "/models/deployed/wrist_ext_right_stage2.stl", color: "#222", opacity: 0.95 },
  { url: "/models/deployed/wrist_ext_right_stage3.stl", color: "#222", opacity: 0.95 },
  { url: "/models/deployed/wrist_ext_left_stage1.stl", color: "#222", opacity: 0.95 },
  { url: "/models/deployed/wrist_ext_left_stage2.stl", color: "#222", opacity: 0.95 },
  { url: "/models/deployed/wrist_ext_left_stage3.stl", color: "#222", opacity: 0.95 },
  { url: "/models/deployed/skin_right.stl", color: "#79a8ff", opacity: 0.45 },
  { url: "/models/deployed/skin_left.stl", color: "#79a8ff", opacity: 0.45 },
];

function StlMesh({
  url,
  color,
  opacity,
  visible,
}: {
  url: string;
  color: string;
  opacity: number;
  visible: boolean;
}) {
  const geom = useLoader(STLLoader, url) as THREE.BufferGeometry;
  return (
    <mesh geometry={geom} visible={visible} castShadow receiveShadow>
      <meshStandardMaterial
        color={color}
        opacity={opacity}
        transparent={opacity < 1}
        metalness={0.1}
        roughness={0.55}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
}

function Scene({ deployState }: { deployState: number }) {
  // Crossfade: opacity weights interpolate between stowed and deployed
  // At s = 0.0 → show stowed (alpha = 1, deployed alpha = 0)
  // At s = 1.0 → show deployed (alpha = 1, stowed alpha = 0)
  // To avoid double-rendering everything on top of itself, we hard-switch
  // at s = 0.5 with a 0.10-wide crossfade band.
  const showStowed = deployState < 0.55;
  const showDeployed = deployState > 0.45;

  // Within the crossfade band, mix the alphas for a smooth visual
  let stowedAlpha = 1.0;
  let deployedAlpha = 1.0;
  if (deployState < 0.45) {
    deployedAlpha = 0.0;
  } else if (deployState > 0.55) {
    stowedAlpha = 0.0;
  } else {
    const t = (deployState - 0.45) / 0.10;
    stowedAlpha = 1 - t;
    deployedAlpha = t;
  }

  return (
    <>
      <hemisphereLight args={["#fff8e6", "#1a2138", 0.65]} />
      <directionalLight
        position={[3, 5, 4]}
        intensity={1.2}
        castShadow
        shadow-mapSize-width={1024}
        shadow-mapSize-height={1024}
      />
      <directionalLight position={[-2, -3, 1]} intensity={0.25} />

      <group>
        {STOWED.map((p) => (
          <StlMesh
            key={`s_${p.url}`}
            url={p.url}
            color={p.color}
            opacity={p.opacity * stowedAlpha}
            visible={showStowed && stowedAlpha > 0.01}
          />
        ))}
        {DEPLOYED.map((p) => (
          <StlMesh
            key={`d_${p.url}`}
            url={p.url}
            color={p.color}
            opacity={p.opacity * deployedAlpha}
            visible={showDeployed && deployedAlpha > 0.01}
          />
        ))}
      </group>

      <gridHelper
        args={[10, 20, "#1a3a8e", "#222"]}
        position={[0, -0.40, 0]}
      />
    </>
  );
}

type ViewerProps = {
  height?: number;
};

const PLAY_DURATION_MS = 2000;

export default function Viewer({ height = 620 }: ViewerProps) {
  const [deployState, setDeployState] = useState(1.0);
  const [playing, setPlaying] = useState(false);
  const [autoRotate, setAutoRotate] = useState(false);

  useEffect(() => {
    if (!playing) return;
    let raf = 0;
    const start = performance.now();
    setDeployState(0);
    const tick = () => {
      const t = (performance.now() - start) / PLAY_DURATION_MS;
      if (t >= 1) {
        setDeployState(1);
        setPlaying(false);
      } else {
        const eased = t * t * (3 - 2 * t); // smooth cubic
        setDeployState(eased);
        raf = requestAnimationFrame(tick);
      }
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [playing]);

  const phaseLabel = (() => {
    if (deployState < 0.05) return "STOWED";
    if (deployState < 0.50) return "Phase A — arms + legs spreading";
    if (deployState < 0.70) return "Phase B — tip extensions firing";
    if (deployState < 0.95) return "Phase D — skin tensioning";
    return "DEPLOYED — glide";
  })();

  return (
    <div className="relative w-full" style={{ height }}>
      <Canvas
        shadows
        camera={{ position: [4.0, 2.6, 4.6], fov: 38, near: 0.05, far: 60 }}
        onCreated={({ scene }) => {
          // Body frame is z-up (our CAD convention); Three.js camera is y-up.
          // Rotate the entire scene root so vehicle z aligns with viewport y.
          scene.rotation.x = -Math.PI / 2;
          scene.background = new THREE.Color("#0a0a0c");
        }}
      >
        <Suspense
          fallback={
            <Html center>
              <div className="text-zinc-300 text-sm">Loading model…</div>
            </Html>
          }
        >
          <Scene deployState={deployState} />
        </Suspense>
        <OrbitControls
          enableDamping
          dampingFactor={0.06}
          autoRotate={autoRotate}
          autoRotateSpeed={0.45}
          minDistance={1.2}
          maxDistance={20}
          target={[-0.3, 0, 0]}
        />
      </Canvas>

      <div className="absolute top-3 right-3 flex flex-col gap-2 max-w-[220px]">
        <button
          onClick={() => setPlaying(true)}
          disabled={playing}
          className="rounded-md bg-orange-500/90 hover:bg-orange-500 disabled:opacity-40 border border-orange-400 px-3 py-1.5 text-xs text-zinc-950 font-medium transition"
        >
          {playing ? "Deploying…" : "▶ Play deployment"}
        </button>
        <button
          onClick={() => {
            setPlaying(false);
            setDeployState(0);
          }}
          className="rounded-md bg-zinc-900/80 hover:bg-zinc-800 border border-zinc-700 px-3 py-1.5 text-xs text-zinc-200 transition"
        >
          ⤴ Stowed (0%)
        </button>
        <button
          onClick={() => {
            setPlaying(false);
            setDeployState(1);
          }}
          className="rounded-md bg-zinc-900/80 hover:bg-zinc-800 border border-zinc-700 px-3 py-1.5 text-xs text-zinc-200 transition"
        >
          ⤵ Deployed (100%)
        </button>
        <button
          onClick={() => setAutoRotate((v) => !v)}
          className="rounded-md bg-zinc-900/80 hover:bg-zinc-800 border border-zinc-700 px-3 py-1.5 text-xs text-zinc-200 transition"
        >
          {autoRotate ? "Stop orbit" : "Auto-orbit"}
        </button>
      </div>

      <div className="absolute bottom-3 left-3 right-3 flex items-center gap-3 bg-zinc-900/85 border border-zinc-800 px-3 py-2 rounded-md text-xs">
        <span className="text-zinc-500 whitespace-nowrap">deploy</span>
        <input
          type="range"
          min={0}
          max={1}
          step={0.01}
          value={deployState}
          onChange={(e) => {
            setDeployState(parseFloat(e.target.value));
            setPlaying(false);
          }}
          className="flex-1 accent-orange-500"
          aria-label="deployment state"
        />
        <span className="mono text-zinc-300 tabular-nums w-10 text-right">
          {(deployState * 100).toFixed(0)}%
        </span>
        <span className="hidden md:inline text-zinc-400 ml-2 text-[11px] uppercase tracking-wider">
          {phaseLabel}
        </span>
      </div>
      <div className="absolute top-3 left-3 text-[11px] text-zinc-500 max-w-[300px]">
        drag to orbit · scroll to zoom · arms extend then tip booms snap out
      </div>
    </div>
  );
}
