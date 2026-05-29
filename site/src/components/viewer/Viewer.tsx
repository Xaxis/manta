import { Suspense, useEffect, useMemo, useRef, useState } from "react";
import { Canvas, useFrame, useLoader } from "@react-three/fiber";
import { OrbitControls, Html, ContactShadows, Environment } from "@react-three/drei";
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";

// ---------------------------------------------------------------
// MANTA viewer — plays the single animated GLB baked from the
// deploy schedule.  The deployment is REAL morph-target animation
// (60 frames); the slider scrubs the AnimationMixer clock.  A
// "Flow field" mode reveals the baked aero field: the wing surface
// coloured by pressure coefficient + animated streamlines, derived
// from the Weissinger span-loading (illustrative, not CFD).
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

// Animated streamline shader: moving pulses along uv.x, coloured by speed (uv.y)
function makeFlowMaterial() {
  return new THREE.ShaderMaterial({
    transparent: true,
    depthWrite: false,
    uniforms: { uTime: { value: 0 } },
    vertexShader: `
      varying vec2 vUv;
      void main() {
        vUv = uv;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
      }
    `,
    fragmentShader: `
      uniform float uTime;
      varying vec2 vUv;
      vec3 ramp(float t) {
        vec3 slow = vec3(0.18, 0.42, 0.98);
        vec3 mid  = vec3(0.20, 0.95, 0.62);
        vec3 fast = vec3(1.00, 0.52, 0.12);
        return t < 0.5 ? mix(slow, mid, t * 2.0) : mix(mid, fast, (t - 0.5) * 2.0);
      }
      void main() {
        float speed = clamp(vUv.y, 0.0, 1.0);
        float pulse = fract(vUv.x * 6.0 - uTime * (0.55 + speed * 1.5));
        float bright = smoothstep(0.0, 0.12, pulse) * (1.0 - smoothstep(0.32, 0.62, pulse));
        vec3 col = ramp(speed) * (0.30 + 1.05 * bright);
        gl_FragColor = vec4(col, 0.30 + 0.6 * bright);
      }
    `,
  });
}

function MantaModel({
  deployRef,
  flowMode,
}: {
  deployRef: React.MutableRefObject<number>;
  flowMode: boolean;
}) {
  const gltf = useLoader(GLTFLoader, MODEL_URL);
  const flowMat = useMemo(() => makeFlowMaterial(), []);

  const { mixer, duration, parts } = useMemo(() => {
    const mx = new THREE.AnimationMixer(gltf.scene);
    const clip = gltf.animations[0];
    if (clip) mx.clipAction(clip).play();

    const parts: {
      pressure?: THREE.Object3D;
      flow?: THREE.Object3D;
      skinMats: THREE.MeshStandardMaterial[];
    } = { skinMats: [] };

    gltf.scene.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (o.name === "PRESSURE") parts.pressure = o;
      if (o.name === "FLOW") parts.flow = o;
      if (mesh.isMesh) {
        mesh.castShadow = true;
        mesh.frustumCulled = false;
        const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
        mats.forEach((mm) => {
          const sm = mm as THREE.MeshStandardMaterial;
          if (sm && sm.name === "skin") parts.skinMats.push(sm);
        });
        // swap the flow streamlines onto the animated shader
        if (o.name === "FLOW") mesh.material = flowMat;
      }
    });
    return { mixer: mx, duration: clip ? clip.duration : 1, parts };
  }, [gltf, flowMat]);

  // toggle the aero field on/off
  useEffect(() => {
    if (parts.pressure) parts.pressure.visible = flowMode;
    if (parts.flow) parts.flow.visible = flowMode;
    parts.skinMats.forEach((sm) => {
      sm.transparent = true;
      sm.opacity = flowMode ? 0.06 : 0.32;
      sm.needsUpdate = true;
    });
  }, [flowMode, parts]);

  useFrame((_, dt) => {
    const t = THREE.MathUtils.clamp(deployRef.current, 0, 1) * duration;
    mixer.setTime(t);
    flowMat.uniforms.uTime.value += dt;
  });

  return <primitive object={gltf.scene} />;
}

function Scene({
  deployRef,
  flowMode,
}: {
  deployRef: React.MutableRefObject<number>;
  flowMode: boolean;
}) {
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
        <MantaModel deployRef={deployRef} flowMode={flowMode} />
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

function easeDeploy(t: number): number {
  if (t < 0.55) {
    const tt = t / 0.55;
    return tt * tt * (3 - 2 * tt) * 0.45;
  }
  const tt = (t - 0.55) / 0.45;
  return 0.45 + tt * tt * (3 - 2 * tt) * 0.55;
}

export default function Viewer({ height = 620 }: { height?: number }) {
  const [deployState, setDeployState] = useState(1.0);
  const [playing, setPlaying] = useState(false);
  const [autoRotate, setAutoRotate] = useState(true);
  const [flowMode, setFlowMode] = useState(false);
  const [telem, setTelem] = useState<Telemetry | null>(null);
  const deployRef = useRef(1.0);

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

  // the aero field is only meaningful on the fully-deployed wing
  const enterFlow = () => {
    setPlaying(false);
    setDeployState(1);
    setFlowMode(true);
  };

  const phaseLabel = (() => {
    if (flowMode) return "AERO FIELD — surface Cp + streamlines (Weissinger span-load · illustrative, not CFD)";
    if (deployState < 0.04) return "STOWED — wingsuit-tucked freefall posture";
    if (deployState < 0.46) return "PHASE A — pneumatic yokes spread arms + legs, lock spars";
    if (deployState < 0.96) return "PHASE B — CO₂ fires; wrist + ankle tip booms telescope out";
    return "DEPLOYED — skin tensioned · FCS captures best-glide trim";
  })();

  const live = useMemo(() => {
    if (!telem) return null;
    const d = telem.series.deploy;
    let idx = 0;
    for (let i = 0; i < d.length; i++) {
      if (d[i] <= deployState) idx = i;
      else break;
    }
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
        <Scene deployRef={deployRef} flowMode={flowMode} />
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
          onClick={() => { setFlowMode(false); setPlaying(true); }}
          disabled={playing}
          className="rounded-md bg-orange-500/90 hover:bg-orange-500 disabled:opacity-40 border border-orange-400 px-3 py-1.5 text-xs text-zinc-950 font-medium transition"
        >
          {playing ? "Deploying…" : "▶ Play deployment"}
        </button>
        <button
          onClick={() => (flowMode ? setFlowMode(false) : enterFlow())}
          className={
            flowMode
              ? "rounded-md bg-sky-500/90 hover:bg-sky-500 border border-sky-400 px-3 py-1.5 text-xs text-zinc-950 font-medium transition"
              : btn
          }
        >
          {flowMode ? "✓ Flow field ON" : "≈ Flow field"}
        </button>
        <button onClick={() => { setFlowMode(false); setPlaying(false); setDeployState(0); }} className={btn}>
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
      {live && !flowMode && (
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

      {/* pressure legend (flow mode) */}
      {flowMode && (
        <div className="absolute top-3 left-3 flex flex-col gap-1.5 bg-zinc-950/75 border border-zinc-800 rounded-md px-3 py-2 text-[11px] backdrop-blur-sm max-w-[230px]">
          <div className="text-zinc-400 uppercase tracking-wider text-[10px]">
            surface pressure · Cp
          </div>
          <div className="h-2 w-full rounded"
               style={{ background: "linear-gradient(90deg,#1a33d9,#19bff2,#34d94d,#fad926,#f22620)" }} />
          <div className="flex justify-between text-[10px] text-zinc-500">
            <span>suction (lift)</span><span>stagnation</span>
          </div>
          <div className="text-[10px] text-zinc-500 leading-snug pt-1">
            streamlines: upwash → suction-peak acceleration → downwash. Field from
            Weissinger span-load + thin-airfoil Cp — illustrative, not CFD.
          </div>
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
          disabled={flowMode}
          onChange={(e) => {
            setDeployState(parseFloat(e.target.value));
            setPlaying(false);
          }}
          className="flex-1 accent-orange-500 disabled:opacity-40"
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
        morph-target animation · locked planform (S 8.4 m² · b 7.4 m · AR 6.5) · drag orbit · scroll zoom
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
