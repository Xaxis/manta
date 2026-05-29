import { Suspense, useEffect, useMemo, useRef, useState } from "react";
import { Canvas, useFrame, useLoader } from "@react-three/fiber";
import { OrbitControls, Html, ContactShadows, Environment } from "@react-three/drei";
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";

// ---------------------------------------------------------------
// MANTA viewer — plays the single animated GLB baked from MuJoCo
// kinematics.  The deployment is REAL morph-target animation (60
// frames); the slider scrubs the AnimationMixer's clock, so there
// is no opacity crossfade / ghosting.  A telemetry strip mirrors
// the verified flight-dynamics physics.
// ---------------------------------------------------------------

const MODEL_URL = "/models/v3/manta.glb";
const TELEMETRY_URL = "/models/v3/telemetry.json";

type Telemetry = {
  deploy_start: number;
  deploy_dur: number;
  settle: {
    V_glide: number;
    glide_ratio: number;
    alpha_glide_deg: number;
    sink_rate: number;
    peak_load: number;
  };
  target: {
    V_best_glide: number;
    L_over_D_max: number;
    V_stall: number;
    V_terminal_stowed: number;
  };
  series: {
    deploy: number[];
    V: number[];
    gamma_deg: number[];
    alpha_deg: number[];
    n_load: number[];
    glide_ratio: number[];
  };
};

function MantaModel({ deployRef }: { deployRef: React.MutableRefObject<number> }) {
  const gltf = useLoader(GLTFLoader, MODEL_URL);

  const { mixer, duration } = useMemo(() => {
    const mx = new THREE.AnimationMixer(gltf.scene);
    const clip = gltf.animations[0];
    if (clip) {
      const action = mx.clipAction(clip);
      action.play();
    }
    return { mixer: mx, duration: clip ? clip.duration : 1 };
  }, [gltf]);

  // Scrub the animation clock from the slider — continuous deployment.
  useFrame(() => {
    const t = THREE.MathUtils.clamp(deployRef.current, 0, 1) * duration;
    mixer.setTime(t);
  });

  useEffect(() => {
    gltf.scene.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (mesh.isMesh) {
        mesh.castShadow = true;
        mesh.frustumCulled = false;
      }
    });
  }, [gltf]);

  return <primitive object={gltf.scene} />;
}

function Scene({ deployRef }: { deployRef: React.MutableRefObject<number> }) {
  return (
    <>
      <hemisphereLight args={["#dfe9ff", "#10131c", 0.55]} />
      <directionalLight
        position={[5, 8, 4]}
        intensity={2.4}
        castShadow
        shadow-mapSize-width={2048}
        shadow-mapSize-height={2048}
        shadow-camera-near={0.5}
        shadow-camera-far={30}
        shadow-camera-left={-6}
        shadow-camera-right={6}
        shadow-camera-top={6}
        shadow-camera-bottom={-6}
      />
      <directionalLight position={[-4, 3, -5]} intensity={0.45} color="#9fb8ff" />

      <Suspense
        fallback={
          <Html center>
            <div className="text-zinc-300 text-sm">Loading model…</div>
          </Html>
        }
      >
        <MantaModel deployRef={deployRef} />
        <Environment preset="city" />
      </Suspense>

      <ContactShadows
        position={[0, -1.05, 0]}
        opacity={0.45}
        scale={14}
        blur={2.4}
        far={4}
        color="#000010"
      />
      <gridHelper args={[14, 28, "#1a3a8e", "#1d2230"]} position={[0, -1.05, 0]} />
    </>
  );
}

const PLAY_DURATION_MS = 2600;

// Eased deploy schedule: slow Phase-A spread, then a fast Phase-B tip snap.
function easeDeploy(t: number): number {
  if (t < 0.55) {
    // Phase A — smooth arm/leg spread over 0..0.4 deploy
    const tt = t / 0.55;
    return (tt * tt * (3 - 2 * tt)) * 0.45;
  }
  // Phase B — sharp tip-extension snap from 0.45 -> 1.0
  const tt = (t - 0.55) / 0.45;
  return 0.45 + tt * tt * (3 - 2 * tt) * 0.55;
}

export default function Viewer({ height = 620 }: { height?: number }) {
  const [deployState, setDeployState] = useState(1.0);
  const [playing, setPlaying] = useState(false);
  const [autoRotate, setAutoRotate] = useState(true);
  const [telem, setTelem] = useState<Telemetry | null>(null);
  const deployRef = useRef(1.0);

  // keep the ref in sync so useFrame reads the latest without re-mounting
  deployRef.current = deployState;

  useEffect(() => {
    fetch(TELEMETRY_URL)
      .then((r) => r.json())
      .then(setTelem)
      .catch(() => {});
  }, []);

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
        setDeployState(easeDeploy(t));
        raf = requestAnimationFrame(tick);
      }
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [playing]);

  const phaseLabel = (() => {
    if (deployState < 0.04) return "STOWED — wingsuit-tucked freefall posture";
    if (deployState < 0.46) return "PHASE A — pneumatic yokes spread arms + legs, lock spars";
    if (deployState < 0.96) return "PHASE B — CO₂ fires; wrist + ankle tip booms telescope out";
    return "DEPLOYED — skin tensioned · FCS captures best-glide trim";
  })();

  // Live telemetry sampled at the current deployment progress.
  const live = useMemo(() => {
    if (!telem) return null;
    const d = telem.series.deploy;
    // find the deploy ramp window and map deployState onto it
    let idx = 0;
    for (let i = 0; i < d.length; i++) {
      if (d[i] <= deployState) idx = i;
      else break;
    }
    // before deploy completes show the deploy-window sample; once fully
    // deployed, show the settled-glide values
    if (deployState >= 0.99) {
      return {
        V: telem.settle.V_glide,
        gamma: -Math.atan2(1, telem.settle.glide_ratio) * (180 / Math.PI),
        n: 1.0,
        ld: telem.settle.glide_ratio,
      };
    }
    return {
      V: telem.series.V[idx],
      gamma: telem.series.gamma_deg[idx],
      n: telem.series.n_load[idx],
      ld: telem.series.glide_ratio[idx],
    };
  }, [telem, deployState]);

  const btn =
    "rounded-md bg-zinc-900/80 hover:bg-zinc-800 border border-zinc-700 px-3 py-1.5 text-xs text-zinc-200 transition";

  return (
    <div className="relative w-full" style={{ height }}>
      <Canvas
        shadows
        dpr={[1, 2]}
        camera={{ position: [4.2, 2.4, 5.2], fov: 34, near: 0.05, far: 80 }}
        onCreated={({ scene }) => {
          scene.background = new THREE.Color("#070809");
        }}
      >
        <Scene deployRef={deployRef} />
        <OrbitControls
          enableDamping
          dampingFactor={0.06}
          autoRotate={autoRotate}
          autoRotateSpeed={0.5}
          minDistance={2.0}
          maxDistance={24}
          target={[0, 0, 0]}
        />
      </Canvas>

      {/* controls */}
      <div className="absolute top-3 right-3 flex flex-col gap-2 max-w-[220px]">
        <button
          onClick={() => setPlaying(true)}
          disabled={playing}
          className="rounded-md bg-orange-500/90 hover:bg-orange-500 disabled:opacity-40 border border-orange-400 px-3 py-1.5 text-xs text-zinc-950 font-medium transition"
        >
          {playing ? "Deploying…" : "▶ Play deployment"}
        </button>
        <button onClick={() => { setPlaying(false); setDeployState(0); }} className={btn}>
          ⤴ Stowed (0%)
        </button>
        <button onClick={() => { setPlaying(false); setDeployState(1); }} className={btn}>
          ⤵ Deployed (100%)
        </button>
        <button onClick={() => setAutoRotate((v) => !v)} className={btn}>
          {autoRotate ? "Stop orbit" : "Auto-orbit"}
        </button>
      </div>

      {/* telemetry strip (verified flight-dynamics physics) */}
      {live && (
        <div className="absolute top-3 left-3 flex flex-col gap-1.5 bg-zinc-950/70 border border-zinc-800 rounded-md px-3 py-2 text-[11px] backdrop-blur-sm">
          <div className="text-zinc-500 uppercase tracking-wider text-[10px]">
            flight physics · live
          </div>
          <Telem label="airspeed" value={`${live.V.toFixed(1)} m/s`} />
          <Telem label="flight path γ" value={`${live.gamma.toFixed(1)}°`} />
          <Telem label="load factor" value={`${live.n.toFixed(2)} g`} />
          <Telem
            label="glide ratio"
            value={live.ld > 0 && live.ld < 40 ? `${live.ld.toFixed(1)} : 1` : "—"}
          />
        </div>
      )}

      {/* deploy scrubber */}
      <div className="absolute bottom-3 left-3 right-3 flex items-center gap-3 bg-zinc-900/85 border border-zinc-800 px-3 py-2 rounded-md text-xs">
        <span className="text-zinc-500 whitespace-nowrap">deploy</span>
        <input
          type="range"
          min={0}
          max={1}
          step={0.005}
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

      <div className="absolute bottom-[3.4rem] left-3 text-[10px] text-zinc-600">
        morph-target animation · baked from MuJoCo · drag orbit · scroll zoom
      </div>
    </div>
  );
}

function Telem({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex items-center justify-between gap-4">
      <span className="text-zinc-500">{label}</span>
      <span className="mono text-zinc-200 tabular-nums">{value}</span>
    </div>
  );
}
