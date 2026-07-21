import React from "react";
import {
  AbsoluteFill,
  useCurrentFrame,
  useVideoConfig,
  spring,
  interpolate,
} from "remotion";
import { COLORS, FONT } from "../theme";

const BENCHMARKS = [
  {
    label: "8192-sphere rain",
    bars: [
      { name: "CPU 1T", ms: 17.99, color: COLORS.gray },
      { name: "CPU 16T", ms: 7.01, color: COLORS.cyan },
      { name: "CUDA", ms: 8.83, color: COLORS.green },
    ],
  },
  {
    label: "2048 spheres + 20k-tri terrain",
    bars: [
      { name: "CPU 1T", ms: 0.84, color: COLORS.gray },
      { name: "CPU 16T", ms: 0.66, color: COLORS.cyan },
      { name: "CUDA", ms: 3.27, color: COLORS.green },
    ],
  },
];

const MAX_MS = 18;

export const BenchmarkScene: React.FC = () => {
  const frame = useCurrentFrame();
  const { fps } = useVideoConfig();

  const headerSpring = spring({ frame, fps, config: { damping: 15 } });

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
        BENCHMARKS
      </div>
      <div
        style={{
          fontFamily: FONT.mono,
          fontSize: 20,
          color: COLORS.gray,
          marginTop: 8,
          opacity: interpolate(frame, [8, 20], [0, 1], {
            extrapolateLeft: "clamp",
            extrapolateRight: "clamp",
          }),
        }}
      >
        RTX 5080 · 60 Hz steps · 4 solver substeps · median ms/step
      </div>

      {BENCHMARKS.map((bench, bi) => {
        const groupDelay = 25 + bi * 80;
        const groupSpring = spring({
          frame: frame - groupDelay,
          fps,
          config: { damping: 16 },
        });

        return (
          <div
            key={bi}
            style={{
              width: 1400,
              marginTop: bi === 0 ? 50 : 40,
              opacity: groupSpring,
              transform: `translateY(${interpolate(groupSpring, [0, 1], [30, 0])}px)`,
            }}
          >
            {/* Scene label */}
            <div
              style={{
                fontFamily: FONT.body,
                fontSize: 26,
                fontWeight: 700,
                color: COLORS.white,
                marginBottom: 18,
              }}
            >
              {bench.label}
            </div>

            {/* Bars */}
            {bench.bars.map((bar, i) => {
              const barDelay = groupDelay + 10 + i * 12;
              const barSpring = spring({
                frame: frame - barDelay,
                fps,
                config: { damping: 18, stiffness: 100 },
              });
              const barWidth = (bar.ms / MAX_MS) * 1100;
              const animatedWidth = barWidth * barSpring;

              const countUp = interpolate(
                frame,
                [barDelay, barDelay + 25],
                [0, bar.ms],
                { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
              );

              return (
                <div
                  key={i}
                  style={{
                    display: "flex",
                    alignItems: "center",
                    marginBottom: 12,
                    height: 44,
                  }}
                >
                  {/* Label */}
                  <div
                    style={{
                      width: 140,
                      fontFamily: FONT.mono,
                      fontSize: 18,
                      color: bar.color,
                      textAlign: "right",
                      paddingRight: 16,
                      fontWeight: 600,
                    }}
                  >
                    {bar.name}
                  </div>

                  {/* Bar */}
                  <div
                    style={{
                      width: animatedWidth,
                      height: 36,
                      backgroundColor: `${bar.color}33`,
                      border: `1px solid ${bar.color}66`,
                      borderRadius: 6,
                      minWidth: barSpring > 0.1 ? 8 : 0,
                    }}
                  />

                  {/* Value — always to the right of the bar */}
                  <span
                    style={{
                      fontFamily: FONT.mono,
                      fontSize: 20,
                      fontWeight: 700,
                      color: bar.color,
                      marginLeft: 14,
                      whiteSpace: "nowrap",
                    }}
                  >
                    {countUp.toFixed(2)} ms
                  </span>
                </div>
              );
            })}
          </div>
        );
      })}

      {/* Speedup callout */}
      <div
        style={{
          marginTop: 40,
          fontFamily: FONT.heading,
          fontSize: 36,
          fontWeight: 800,
          color: COLORS.cyan,
          opacity: interpolate(frame, [200, 215], [0, 1], {
            extrapolateLeft: "clamp",
            extrapolateRight: "clamp",
          }),
          transform: `scale(${interpolate(frame, [200, 215], [0.9, 1], {
            extrapolateLeft: "clamp",
            extrapolateRight: "clamp",
          })})`,
        }}
      >
        2.6× faster with 16 threads · GPU scales to 4096+ joints
      </div>
    </AbsoluteFill>
  );
};
