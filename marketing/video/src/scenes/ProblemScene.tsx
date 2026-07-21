import React from "react";
import {
  AbsoluteFill,
  useCurrentFrame,
  useVideoConfig,
  spring,
  interpolate,
} from "remotion";
import { COLORS, FONT } from "../theme";

const Wall: React.FC<{ x: number }> = ({ x }) => (
  <div
    style={{
      position: "absolute",
      left: x,
      top: 100,
      width: 30,
      height: 300,
      backgroundColor: COLORS.grayDark,
      borderRadius: 4,
      border: `1px solid ${COLORS.gray}`,
    }}
  />
);

const Bullet: React.FC<{ x: number; y: number; ghost?: boolean }> = ({
  x,
  y,
  ghost,
}) => (
  <div
    style={{
      position: "absolute",
      left: x,
      top: y,
      width: 24,
      height: 10,
      borderRadius: 5,
      backgroundColor: ghost ? `${COLORS.orange}44` : COLORS.orange,
      boxShadow: ghost ? "none" : `0 0 12px ${COLORS.orange}88`,
    }}
  />
);

export const ProblemScene: React.FC = () => {
  const frame = useCurrentFrame();
  const { fps } = useVideoConfig();

  const headerSpring = spring({
    frame,
    fps,
    config: { damping: 15 },
  });

  // Bullet travels from left, passes through wall
  const bulletX = interpolate(frame, [30, 90], [200, 1400], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
    easing: (t) => t, // linear = fast bullet
  });

  const wallX = 900;
  const bulletY = 240;
  const passedThrough = bulletX > wallX + 40;

  // Red X appears when bullet passes
  const xOpacity = interpolate(frame, [95, 105], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  // Problem texts stagger in
  const problems = [
    "Bullets tunnel through walls",
    "Event-driven CCD stalls on piles",
    "Speculative contacts leak at extremes",
  ];

  return (
    <AbsoluteFill
      style={{
        backgroundColor: COLORS.bg,
        justifyContent: "flex-start",
        alignItems: "center",
        paddingTop: 80,
      }}
    >
      {/* Header */}
      <div
        style={{
          fontFamily: FONT.heading,
          fontSize: 64,
          fontWeight: 800,
          color: COLORS.red,
          letterSpacing: 8,
          opacity: headerSpring,
          transform: `translateY(${interpolate(headerSpring, [0, 1], [-30, 0])}px)`,
        }}
      >
        THE PROBLEM
      </div>

      {/* Simulation area */}
      <div
        style={{
          position: "relative",
          width: 1600,
          height: 500,
          marginTop: 60,
          backgroundColor: COLORS.bgLight,
          borderRadius: 16,
          border: `1px solid ${COLORS.grayDark}`,
          overflow: "hidden",
        }}
      >
        {/* Label */}
        <div
          style={{
            position: "absolute",
            top: 16,
            left: 24,
            fontFamily: FONT.mono,
            fontSize: 18,
            color: COLORS.gray,
          }}
        >
          naive integration @ 2 km/s
        </div>

        <Wall x={wallX} />

        {/* Ghost trail */}
        {passedThrough &&
          [0, 1, 2].map((i) => (
            <Bullet
              key={i}
              x={bulletX - 80 - i * 60}
              y={bulletY}
              ghost
            />
          ))}

        <Bullet x={bulletX} y={bulletY} />

        {/* Wall label */}
        <div
          style={{
            position: "absolute",
            left: wallX - 20,
            top: 410,
            fontFamily: FONT.mono,
            fontSize: 16,
            color: COLORS.gray,
          }}
        >
          wall
        </div>

        {/* Red X overlay */}
        <div
          style={{
            position: "absolute",
            top: 0,
            left: 0,
            right: 0,
            bottom: 0,
            display: "flex",
            justifyContent: "center",
            alignItems: "center",
            opacity: xOpacity,
          }}
        >
          <span
            style={{
              fontSize: 200,
              color: COLORS.red,
              fontWeight: 800,
              fontFamily: FONT.heading,
              textShadow: `0 0 60px ${COLORS.red}66`,
            }}
          >
            ✕
          </span>
        </div>
      </div>

      {/* Problem list */}
      <div style={{ marginTop: 50, display: "flex", flexDirection: "column", gap: 18 }}>
        {problems.map((text, i) => {
          const itemOpacity = interpolate(
            frame,
            [105 + i * 12, 115 + i * 12],
            [0, 1],
            { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
          );
          return (
            <div
              key={i}
              style={{
                fontFamily: FONT.body,
                fontSize: 30,
                color: COLORS.white,
                opacity: itemOpacity,
                transform: `translateX(${interpolate(
                  frame,
                  [105 + i * 12, 115 + i * 12],
                  [-20, 0],
                  { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
                )}px)`,
                display: "flex",
                alignItems: "center",
                gap: 14,
              }}
            >
              <span style={{ color: COLORS.red, fontSize: 22 }}>●</span>
              {text}
            </div>
          );
        })}
      </div>
    </AbsoluteFill>
  );
};
