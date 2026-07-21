import React from "react";
import {
  AbsoluteFill,
  useCurrentFrame,
  useVideoConfig,
  spring,
  interpolate,
  Easing,
} from "remotion";
import { COLORS, FONT } from "../theme";

const TITLE = "VELOX";
const SUBTITLE = "Fast · Tunneling-Resistant · GPU-Accelerated";
const TAGLINE = "A 3D physics engine for games — C++17";

export const TitleScene: React.FC = () => {
  const frame = useCurrentFrame();
  const { fps } = useVideoConfig();

  const letters = TITLE.split("");

  return (
    <AbsoluteFill
      style={{
        backgroundColor: COLORS.bg,
        justifyContent: "center",
        alignItems: "center",
      }}
    >
      {/* Speed lines background */}
      {Array.from({ length: 12 }).map((_, i) => {
        const lineProgress = interpolate(
          frame,
          [5 + i * 2, 35 + i * 2],
          [0, 1],
          { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
        );
        const y = 80 + i * 80;
        return (
          <div
            key={i}
            style={{
              position: "absolute",
              top: y,
              left: 0,
              width: `${lineProgress * 100}%`,
              height: 1,
              background: `linear-gradient(90deg, transparent, ${COLORS.cyan}22, transparent)`,
            }}
          />
        );
      })}

      {/* Main title */}
      <div style={{ display: "flex", gap: 8 }}>
        {letters.map((letter, i) => {
          const s = spring({
            frame: frame - 8 - i * 4,
            fps,
            config: { damping: 12, stiffness: 200, mass: 0.8 },
          });
          return (
            <span
              key={i}
              style={{
                fontFamily: FONT.heading,
                fontSize: 140,
                fontWeight: 800,
                color: COLORS.white,
                letterSpacing: 12,
                transform: `translateY(${interpolate(s, [0, 1], [60, 0])}px) scale(${interpolate(s, [0, 1], [0.7, 1])})`,
                opacity: s,
                textShadow: `0 0 40px ${COLORS.cyan}66, 0 0 80px ${COLORS.cyan}22`,
              }}
            >
              {letter}
            </span>
          );
        })}
      </div>

      {/* Accent line */}
      <div
        style={{
          width: interpolate(
            spring({ frame: frame - 30, fps, config: { damping: 20 } }),
            [0, 1],
            [0, 400]
          ),
          height: 3,
          backgroundColor: COLORS.cyan,
          marginTop: 20,
          marginBottom: 30,
          borderRadius: 2,
          boxShadow: `0 0 20px ${COLORS.cyan}88`,
        }}
      />

      {/* Subtitle */}
      <div
        style={{
          fontFamily: FONT.body,
          fontSize: 36,
          color: COLORS.cyan,
          fontWeight: 600,
          letterSpacing: 6,
          opacity: interpolate(frame, [40, 55], [0, 1], {
            extrapolateLeft: "clamp",
            extrapolateRight: "clamp",
          }),
          transform: `translateY(${interpolate(frame, [40, 55], [15, 0], {
            extrapolateLeft: "clamp",
            extrapolateRight: "clamp",
          })}px)`,
        }}
      >
        {SUBTITLE}
      </div>

      {/* Tagline */}
      <div
        style={{
          fontFamily: FONT.body,
          fontSize: 24,
          color: COLORS.gray,
          marginTop: 16,
          opacity: interpolate(frame, [55, 70], [0, 1], {
            extrapolateLeft: "clamp",
            extrapolateRight: "clamp",
          }),
        }}
      >
        {TAGLINE}
      </div>
    </AbsoluteFill>
  );
};
