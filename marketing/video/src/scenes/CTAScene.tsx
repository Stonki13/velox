import React from "react";
import {
  AbsoluteFill,
  useCurrentFrame,
  useVideoConfig,
  spring,
  interpolate,
} from "remotion";
import { COLORS, FONT } from "../theme";

export const CTAScene: React.FC = () => {
  const frame = useCurrentFrame();
  const { fps } = useVideoConfig();

  const logoSpring = spring({
    frame,
    fps,
    config: { damping: 12, stiffness: 150 },
  });

  const fadeOut = interpolate(frame, [180, 210], [1, 0], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  return (
    <AbsoluteFill
      style={{
        backgroundColor: COLORS.bg,
        justifyContent: "center",
        alignItems: "center",
        opacity: fadeOut,
      }}
    >
      {/* Glow behind logo */}
      <div
        style={{
          position: "absolute",
          width: 500,
          height: 500,
          borderRadius: 250,
          background: `radial-gradient(circle, ${COLORS.cyan}11, transparent 70%)`,
        }}
      />

      {/* Logo */}
      <div
        style={{
          fontFamily: FONT.heading,
          fontSize: 120,
          fontWeight: 800,
          color: COLORS.white,
          letterSpacing: 16,
          opacity: logoSpring,
          transform: `scale(${interpolate(logoSpring, [0, 1], [0.8, 1])})`,
          textShadow: `0 0 60px ${COLORS.cyan}44`,
        }}
      >
        VELOX
      </div>

      {/* Accent line */}
      <div
        style={{
          width: interpolate(
            spring({ frame: frame - 12, fps, config: { damping: 20 } }),
            [0, 1],
            [0, 300]
          ),
          height: 3,
          backgroundColor: COLORS.cyan,
          marginTop: 16,
          marginBottom: 30,
          borderRadius: 2,
        }}
      />

      {/* Tagline */}
      <div
        style={{
          fontFamily: FONT.body,
          fontSize: 28,
          color: COLORS.gray,
          opacity: interpolate(frame, [20, 35], [0, 1], {
            extrapolateLeft: "clamp",
            extrapolateRight: "clamp",
          }),
        }}
      >
        MIT Licensed · C++17 · CUDA Optional
      </div>

      {/* GitHub URL */}
      <div
        style={{
          fontFamily: FONT.mono,
          fontSize: 34,
          color: COLORS.cyan,
          marginTop: 30,
          opacity: interpolate(frame, [35, 50], [0, 1], {
            extrapolateLeft: "clamp",
            extrapolateRight: "clamp",
          }),
          transform: `translateY(${interpolate(frame, [35, 50], [10, 0], {
            extrapolateLeft: "clamp",
            extrapolateRight: "clamp",
          })}px)`,
        }}
      >
        github.com/Stonki13/velox
      </div>

      {/* Star CTA */}
      <div
        style={{
          fontFamily: FONT.body,
          fontSize: 26,
          color: COLORS.white,
          marginTop: 24,
          opacity: interpolate(frame, [55, 70], [0, 1], {
            extrapolateLeft: "clamp",
            extrapolateRight: "clamp",
          }),
          backgroundColor: `${COLORS.cyan}22`,
          border: `1px solid ${COLORS.cyan}44`,
          borderRadius: 12,
          padding: "12px 32px",
        }}
      >
        ⭐ Star us on GitHub
      </div>
    </AbsoluteFill>
  );
};
