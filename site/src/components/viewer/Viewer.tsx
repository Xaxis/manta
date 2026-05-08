import { Suspense, useMemo, useRef, useState, useEffect } from "react";
import { Canvas, useLoader, useFrame } from "@react-three/fiber";
import { OrbitControls, Html } from "@react-three/drei";
import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";

// -------------------------------------------------------------
// Per-subsystem definitions: STL paths, colors, animation rules
// -------------------------------------------------------------

type AnimRule = {
  // deploy_state ∈ [0,1]; rules return per-frame transform.
  // For each rule we return a Matrix4 to apply to the mesh.
  visible?: (s: number) => boolean;
  scale?: (s: number) => [number, number, number];
  rotation?: (s: number) => [number, number, number];   // Euler XYZ radians
  position?: (s: number) => [number, number, number];   // translate offset
};

type Part = {
  id: string;
  url: string;
  color: string;
  opacity?: number;
  metalness?: number;
  roughness?: number;
  label: string;
  anim?: AnimRule;
};

// Deployment-state animation choreography.
//   s = 0.00 → 0.10  (stowed; everything close to body, wing/spars hidden)
//   s = 0.10 → 0.30  drogue stable (no visible change in airframe — happens during freefall)
//   s = 0.30 → 0.65  spars sweep + telescope out
//   s = 0.55 → 0.85  ribs unfurl (we don't model ribs explicitly, but wing fades in)
//   s = 0.65 → 1.00  wing skin tensions, full glide
//
// The static body parts (pilot, rig, sub-frame, cutters) are visible the
// whole time. Reserve cone is shown only post-jettison (separate toggle).

const easeInOut = (t: number) => t * t * (3 - 2 * t);

const sparAnim: AnimRule = {
  // Spars start collapsed (small scale along the spar axis) and stowed
  // along +x (pilot body axis), then sweep out and extend.
  // Approximate by:
  //   - rotation about z: from +90° (along +x) to 0° (deployed swept direction)
  //   - scale along the spar's local y axis: 0.35 (collapsed root-stage only)
  //     to 1.0 (fully extended)
  // This is a viewer-side approximation of the deploy kinematics; the real
  // spar telescopes through 3 stages with locking-pin engagements per stage.
  visible: (s) => true,
  scale: (s) => {
    const phase = Math.max(0, Math.min(1, (s - 0.30) / 0.35));
    const f = 0.35 + (1 - 0.35) * easeInOut(phase);
    return [1, f, 1];
  },
  rotation: (s) => {
    // Stowed: spars along +x (rotate by +π/2 - sweep about z); deployed: 0.
    const phase = Math.max(0, Math.min(1, (s - 0.30) / 0.35));
    const angle = (1 - easeInOut(phase)) * (Math.PI / 2 - (22 * Math.PI) / 180);
    return [0, 0, angle];
  },
};

const wingAnim: AnimRule = {
  // Wing OML fades in starting at s = 0.55 (skin starts to take shape) and
  // ends fully present at s = 1.0.
  scale: (s) => {
    const phase = Math.max(0, Math.min(1, (s - 0.55) / 0.45));
    const f = easeInOut(phase);
    return [f, 1, f];
  },
};

const stubAnim: AnimRule = {
  // Stubs are visible only post-jettison. In the deployment animation we
  // don't show them; they stay at unit scale. The reserve-cone toggle
  // path is what shows them.
};

const PARTS: Part[] = [
  {
    id: "pilot",
    url: "/models/parts/pilot.stl",
    color: "#a87d52",
    opacity: 0.85,
    metalness: 0.0,
    roughness: 0.85,
    label: "pilot torso (placeholder volume)",
  },
  {
    id: "rig_main",
    url: "/models/parts/rig_main.stl",
    color: "#3a3a3a",
    opacity: 0.95,
    metalness: 0.05,
    roughness: 0.7,
    label: "main canopy container",
  },
  {
    id: "rig_reserve",
    url: "/models/parts/rig_reserve.stl",
    color: "#a02a2a",
    opacity: 0.95,
    metalness: 0.05,
    roughness: 0.7,
    label: "reserve canopy container",
  },
  {
    id: "subframe",
    url: "/models/parts/subframe.stl",
    color: "#888",
    opacity: 0.95,
    metalness: 0.5,
    roughness: 0.4,
    label: "wing-mount sub-frame",
  },
  {
    id: "wing",
    url: "/models/parts/wing.stl",
    color: "#79a8ff",
    opacity: 0.5,
    metalness: 0.05,
    roughness: 0.4,
    label: "wing OML (DCF skin over ribs)",
    anim: wingAnim,
  },
  {
    id: "front_spar",
    url: "/models/parts/front_spar.stl",
    color: "#1a1a1a",
    opacity: 0.95,
    metalness: 0.6,
    roughness: 0.3,
    label: "front spar (telescoping CFRP, 73 mm OD root)",
    anim: sparAnim,
  },
  {
    id: "rear_spar",
    url: "/models/parts/rear_spar.stl",
    color: "#3a3a3a",
    opacity: 0.95,
    metalness: 0.6,
    roughness: 0.3,
    label: "rear spar (telescoping CFRP, 30 mm OD root)",
    anim: sparAnim,
  },
  {
    id: "stubs",
    url: "/models/parts/stubs.stl",
    color: "#cc2222",
    opacity: 0.9,
    metalness: 0.5,
    roughness: 0.3,
    label: "post-jettison stubs (visible after wing departs)",
    anim: stubAnim,
  },
];

const RESERVE_CONE_PART: Part = {
  id: "reserve_cone",
  url: "/models/parts/reserve_cone.stl",
  color: "#ffaa00",
  opacity: 0.18,
  metalness: 0.0,
  roughness: 0.8,
  label: "reserve canopy deployment cone (30° half-angle)",
};

function StlPart({
  part,
  deployState,
  visible = true,
}: {
  part: Part;
  deployState: number;
  visible?: boolean;
}) {
  const geom = useLoader(STLLoader, part.url) as THREE.BufferGeometry;
  const meshRef = useRef<THREE.Mesh>(null);

  useFrame(() => {
    const m = meshRef.current;
    if (!m) return;
    if (part.anim?.scale) {
      const [sx, sy, sz] = part.anim.scale(deployState);
      m.scale.set(sx, sy, sz);
    } else {
      m.scale.set(1, 1, 1);
    }
    if (part.anim?.rotation) {
      const [rx, ry, rz] = part.anim.rotation(deployState);
      m.rotation.set(rx, ry, rz);
    } else {
      m.rotation.set(0, 0, 0);
    }
    if (part.anim?.position) {
      const [px, py, pz] = part.anim.position(deployState);
      m.position.set(px, py, pz);
    }
  });

  if (!visible) return null;

  return (
    <mesh ref={meshRef} geometry={geom} castShadow receiveShadow>
      <meshStandardMaterial
        color={part.color}
        opacity={part.opacity ?? 1}
        transparent={(part.opacity ?? 1) < 1}
        metalness={part.metalness ?? 0.1}
        roughness={part.roughness ?? 0.55}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
}

function Scene({
  deployState,
  showReserveCone,
  showStubs,
}: {
  deployState: number;
  showReserveCone: boolean;
  showStubs: boolean;
}) {
  // After full jettison (when showReserveCone is on), hide spars+wing and
  // show stubs. Otherwise, hide stubs (they're conceptual, only relevant
  // post-jettison).
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
        {PARTS.map((part) => {
          const isAirframe = ["wing", "front_spar", "rear_spar"].includes(part.id);
          const isStub = part.id === "stubs";
          // Hide airframe in jettison view; show stubs only in jettison view
          const visible = showReserveCone
            ? !isAirframe
            : !isStub || showStubs;
          return (
            <StlPart
              key={part.id}
              part={part}
              deployState={deployState}
              visible={visible}
            />
          );
        })}
        {showReserveCone && (
          <StlPart
            part={RESERVE_CONE_PART}
            deployState={deployState}
            visible={true}
          />
        )}
      </group>

      <gridHelper
        args={[10, 20, "#1a3a8e", "#222"]}
        position={[0, -0.36, 0]}
      />
    </>
  );
}

type ViewerProps = {
  height?: number;
};

export default function Viewer({ height = 620 }: ViewerProps) {
  const [deployState, setDeployState] = useState(1.0);
  const [autoDeploy, setAutoDeploy] = useState(false);
  const [showReserveCone, setShowReserveCone] = useState(false);
  const [showStubs, setShowStubs] = useState(false);
  const [autoRotate, setAutoRotate] = useState(false);

  // Auto-deploy: ramp from 0 → 1 over 4s, then hold
  useEffect(() => {
    if (!autoDeploy) return;
    let raf = 0;
    const start = performance.now();
    setDeployState(0);
    const tick = () => {
      const t = (performance.now() - start) / 4000;
      if (t >= 1) {
        setDeployState(1);
      } else {
        setDeployState(Math.max(0, t));
        raf = requestAnimationFrame(tick);
      }
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [autoDeploy]);

  return (
    <div className="relative w-full" style={{ height }}>
      <Canvas
        shadows
        camera={{ position: [4.2, 3.0, 5.0], fov: 35, near: 0.05, far: 60 }}
        onCreated={({ scene }) => {
          // Body frame is z-up; Three.js default is y-up.
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
          <Scene
            deployState={deployState}
            showReserveCone={showReserveCone}
            showStubs={showStubs}
          />
        </Suspense>
        <OrbitControls
          enableDamping
          dampingFactor={0.06}
          autoRotate={autoRotate}
          autoRotateSpeed={0.5}
          minDistance={1.5}
          maxDistance={20}
          target={[1.0, 0, 0]}
        />
      </Canvas>

      {/* Controls overlay */}
      <div className="absolute top-3 right-3 flex flex-col gap-2 max-w-[220px]">
        <button
          onClick={() => {
            setShowReserveCone(false);
            setAutoDeploy((v) => !v);
            if (autoDeploy) setDeployState(1);
          }}
          className="rounded-md bg-orange-500/90 hover:bg-orange-500 border border-orange-400 px-3 py-1.5 text-xs text-zinc-950 font-medium transition"
        >
          {autoDeploy ? "Stop deploy" : "Play deployment"}
        </button>
        <button
          onClick={() => {
            setShowReserveCone((v) => !v);
            setShowStubs(true);
            setDeployState(1);
          }}
          className="rounded-md bg-zinc-900/80 hover:bg-zinc-800 border border-zinc-700 px-3 py-1.5 text-xs text-zinc-200 transition"
        >
          {showReserveCone ? "Back to deployed" : "Show jettison + reserve cone"}
        </button>
        <button
          onClick={() => setAutoRotate((v) => !v)}
          className="rounded-md bg-zinc-900/80 hover:bg-zinc-800 border border-zinc-700 px-3 py-1.5 text-xs text-zinc-200 transition"
        >
          {autoRotate ? "Stop orbit" : "Auto-orbit"}
        </button>
      </div>

      {/* Deploy slider */}
      <div className="absolute bottom-3 left-3 right-3 flex items-center gap-3 bg-zinc-900/80 border border-zinc-800 px-3 py-2 rounded-md text-xs">
        <span className="text-zinc-500 whitespace-nowrap">deploy</span>
        <input
          type="range"
          min={0}
          max={1}
          step={0.01}
          value={deployState}
          onChange={(e) => {
            setDeployState(parseFloat(e.target.value));
            setAutoDeploy(false);
          }}
          className="w-full accent-orange-500"
          aria-label="deployment state"
        />
        <span className="mono text-zinc-300 tabular-nums w-10 text-right">
          {(deployState * 100).toFixed(0)}%
        </span>
      </div>
      <div className="absolute top-3 left-3 text-[11px] text-zinc-500 max-w-[280px]">
        drag to orbit · scroll to zoom · slider scrubs deployment
      </div>
    </div>
  );
}
