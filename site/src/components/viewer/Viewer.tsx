import { Suspense, useEffect, useState } from "react";
import { Canvas, useLoader } from "@react-three/fiber";
import { OrbitControls, Html } from "@react-three/drei";
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";

// ---------------------------------------------------------------
// MANTA viewer — loads GLB scenes baked from MuJoCo + Blender.
// Three poses: stowed (deploy=0), mid-deploy (~0.5), deployed (1.0).
// Slider crossfades between adjacent poses.
// ---------------------------------------------------------------

type Pose = "stowed" | "mid_deploy" | "deployed";
const POSE_URLS: Record<Pose, string> = {
  stowed: "/models/v3/stowed.glb",
  mid_deploy: "/models/v3/mid_deploy.glb",
  deployed: "/models/v3/deployed.glb",
};

function GltfScene({
  pose,
  visible,
  opacity,
}: {
  pose: Pose;
  visible: boolean;
  opacity: number;
}) {
  const gltf = useLoader(GLTFLoader, POSE_URLS[pose]);
  // Apply the opacity to all materials in the scene
  useEffect(() => {
    gltf.scene.traverse((child) => {
      if ((child as THREE.Mesh).isMesh) {
        const mesh = child as THREE.Mesh;
        const mat = mesh.material as THREE.MeshStandardMaterial;
        if (mat) {
          mat.transparent = opacity < 1.0;
          mat.opacity = opacity;
          mat.depthWrite = opacity > 0.95;
        }
      }
    });
  }, [gltf, opacity]);
  if (!visible) return null;
  return <primitive object={gltf.scene.clone()} />;
}

function Scene({ deployState }: { deployState: number }) {
  // Crossfade across three poses:
  //   0.00 - 0.40   stowed → mid_deploy
  //   0.40 - 1.00   mid_deploy → deployed
  let stowedAlpha = 0.0;
  let midAlpha = 0.0;
  let deployedAlpha = 0.0;

  if (deployState <= 0.40) {
    const t = deployState / 0.40;
    stowedAlpha = 1 - t;
    midAlpha = t;
  } else {
    const t = (deployState - 0.40) / 0.60;
    midAlpha = 1 - t;
    deployedAlpha = t;
  }

  return (
    <>
      <hemisphereLight args={["#fff8e6", "#1a2138", 0.65]} />
      <directionalLight
        position={[3, -5, 6]}
        intensity={2.0}
        castShadow
        shadow-mapSize-width={1024}
        shadow-mapSize-height={1024}
      />
      <directionalLight position={[-3, 4, 4]} intensity={0.5} />

      <Suspense
        fallback={
          <Html center>
            <div className="text-zinc-300 text-sm">Loading scene…</div>
          </Html>
        }
      >
        {stowedAlpha > 0.01 && (
          <GltfScene pose="stowed" visible={true} opacity={stowedAlpha} />
        )}
        {midAlpha > 0.01 && (
          <GltfScene pose="mid_deploy" visible={true} opacity={midAlpha} />
        )}
        {deployedAlpha > 0.01 && (
          <GltfScene pose="deployed" visible={true} opacity={deployedAlpha} />
        )}
      </Suspense>

      <gridHelper
        args={[10, 20, "#1a3a8e", "#222"]}
        position={[0, -0.5, 0]}
      />
    </>
  );
}

type ViewerProps = {
  height?: number;
};

const PLAY_DURATION_MS = 2200;

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
        // Eased: a slow lead-in (Phase A spread) then a fast pulse (Phase B
        // tip-extension fire) then settle.
        let eased: number;
        if (t < 0.5) {
          // Phase A: smooth spread over 0..0.5 of the play duration
          eased = (t / 0.5) * 0.4;
        } else {
          // Phase B: sharp ramp from 0.4 → 1.0 over 0.5..0.7 of play duration
          // then hold settle to 1.0
          const tt = (t - 0.5) / 0.5;
          const sharp = tt * tt * (3 - 2 * tt);
          eased = 0.4 + sharp * 0.6;
        }
        setDeployState(eased);
        raf = requestAnimationFrame(tick);
      }
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [playing]);

  const phaseLabel = (() => {
    if (deployState < 0.05) return "STOWED — wingsuit-tucked posture";
    if (deployState < 0.42) return "Phase A — arms + legs spreading";
    if (deployState < 0.95) return "Phase B — CO₂ fires; tip extensions snap out";
    return "DEPLOYED — wing tensioned, glide";
  })();

  return (
    <div className="relative w-full" style={{ height }}>
      <Canvas
        shadows
        camera={{ position: [3.0, 1.6, 3.8], fov: 32, near: 0.05, far: 60 }}
        onCreated={({ scene }) => {
          // GLB files use Blender's z-up world. Three.js default is y-up.
          // Rotate the scene root so vehicle z aligns with viewport up.
          scene.rotation.x = -Math.PI / 2;
          scene.background = new THREE.Color("#0a0a0c");
        }}
      >
        <Scene deployState={deployState} />
        <OrbitControls
          enableDamping
          dampingFactor={0.06}
          autoRotate={autoRotate}
          autoRotateSpeed={0.45}
          minDistance={1.5}
          maxDistance={20}
          target={[0, 0, 0]}
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
        physics-baked from MuJoCo · drag orbit · scroll zoom
      </div>
    </div>
  );
}
