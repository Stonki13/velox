import React from "react";
import {
  AbsoluteFill,
  useCurrentFrame,
  useVideoConfig,
  spring,
  interpolate,
} from "remotion";
import { COLORS, FONT } from "../theme";

const FEATURES = [
  { icon: "🎯", label: "Predictive Contact Sweeping", sub: "CCD that actually works" },
  { icon: "⚡", label: "GPU Accelerated", sub: "CUDA backend, same code" },
  { icon: "🔧", label: "7 Joint Types + 6DoF", sub: "Motors, limits, breaking" },
  { icon: "🧊", label: "9 Collider Types", sub: "Sphere to convex hull to mesh" },
  { icon: "🧵", label: "Multithreaded CPU", sub: "Deterministic worker pool" },
  { icon: "💾", label: "Rollback Snapshots", sub: "Full world checkpoints" },
  { icon: "🌍", label: "Origin Shifting", sub: "Large-world support" },
  { icon: "🧪", label: "29-Test Suite", sub: "Fuzzing + Jolt diff-tests" },
  { icon: "📐", label: "GJK / EPA", sub: "Shared CPU + CUDA paths" },
  { icon: "🏗️", label: "Ragdoll Builder", sub: "Cone-twist bone trees" },
  { icon: "🚗", label: "Vehicle Model", sub: "Wheeled vehicle support" },
  { icon: "📜", label: "MIT Licensed", sub: "Use it anywhere" },
];

export const FeaturesScene: React.FC = () => {
  const frame = useCurrentFrame();
  const { fps } = useVideoConfig();

  const headerSpring = spring({ frame, fps, config: { damping: 15 } });

  const COLS = 4;
  const ROWS = 3;

  return (
    <AbsoluteFill
      style={{
        backgroundColor: COLORS.bg,
        justifyContent: "flex-start",
        alignItems: "center",
        paddingTop: 70,
      }}
    >
      <div
        style={{
          fontFamily: FONT.heading,
          fontSize: 56,
          fontWeight: 800,
          color: COLORS.white,
          letterSpacing: 6,
          opacity: headerSpring,
        }}
      >
        BATTERIES INCLUDED
      </div>

      <div
        style={{
          display: "grid",
          gridTemplateColumns: `repeat(${COLS}, 1fr)`,
          gap: 20,
          marginTop: 50,
          padding: "0 100px",
          width: "100%",
        }}
      >
        {FEATURES.map((feat, i) => {
          const delay = 15 + i * 8;
          const s = spring({
            frame: frame - delay,
            fps,
            config: { damping: 14, stiffness: 150 },
          });

          return (
            <div
              key={i}
              style={{
                backgroundColor: COLORS.bgLight,
                borderRadius: 14,
                padding: "24px 22px",
                border: `1px solid ${COLORS.grayDark}`,
                opacity: s,
                transform: `translateY(${interpolate(s, [0, 1], [25, 0])}px) scale(${interpolate(s, [0, 1], [0.95, 1])})`,
              }}
            >
              <div style={{ fontSize: 36, marginBottom: 10 }}>{feat.icon}</div>
              <div
                style={{
                  fontFamily: FONT.heading,
                  fontSize: 20,
                  fontWeight: 700,
                  color: COLORS.white,
                  marginBottom: 6,
                }}
              >
                {feat.label}
              </div>
              <div
                style={{
                  fontFamily: FONT.body,
                  fontSize: 16,
                  color: COLORS.gray,
                }}
              >
                {feat.sub}
              </div>
            </div>
          );
        })}
      </div>
    </AbsoluteFill>
  );
};
