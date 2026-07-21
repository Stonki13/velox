import React from "react";
import {
  AbsoluteFill,
  useCurrentFrame,
  useVideoConfig,
  spring,
  interpolate,
} from "remotion";
import { COLORS, FONT } from "../theme";

const steps = [
  {
    num: "1",
    title: "Speculative Detection",
    desc: "Contacts created while pairs are still apart",
    color: COLORS.cyan,
  },
  {
    num: "2",
    title: "Iterative Velocity Solve",
    desc: "Removes only excess approach velocity — grazing bodies keep speed",
    color: COLORS.cyan,
  },
  {
    num: "3",
    title: "Conservative Advancement",
    desc: "Post-integration safety net rewinds to exact time of impact",
    color: COLORS.green,
  },
];

export const SolutionScene: React.FC = () => {
  const frame = useCurrentFrame();
  const { fps } = useVideoConfig();

  const headerSpring = spring({ frame, fps, config: { damping: 15 } });

  // Bullet that stops at wall
  const bulletX = interpolate(frame, [120, 165], [200, 870], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const bulletStopped = frame >= 165;

  // Impact flash
  const flashOpacity = bulletStopped
    ? interpolate(frame, [165, 170, 180], [0.8, 0.4, 0], {
        extrapolateLeft: "clamp",
        extrapolateRight: "clamp",
      })
    : 0;

  const checkOpacity = interpolate(frame, [175, 185], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });

  return (
    <AbsoluteFill
      style={{
        backgroundColor: COLORS.bg,
        justifyContent: "flex-start",
        alignItems: "center",
        paddingTop: 70,
      }}
    >
      {/* Header */}
      <div
        style={{
          fontFamily: FONT.heading,
          fontSize: 56,
          fontWeight: 800,
          color: COLORS.cyan,
          letterSpacing: 4,
          opacity: headerSpring,
          textShadow: `0 0 30px ${COLORS.cyan}44`,
        }}
      >
        PREDICTIVE CONTACT SWEEPING
      </div>
      <div
        style={{
          fontFamily: FONT.body,
          fontSize: 22,
          color: COLORS.gray,
          marginTop: 8,
          opacity: interpolate(frame, [10, 25], [0, 1], {
            extrapolateLeft: "clamp",
            extrapolateRight: "clamp",
          }),
        }}
      >
        Three layers. No tunneling. No stalling. No leaks.
      </div>

      {/* Steps */}
      <div
        style={{
          display: "flex",
          gap: 30,
          marginTop: 50,
          padding: "0 80px",
        }}
      >
        {steps.map((step, i) => {
          const s = spring({
            frame: frame - 25 - i * 25,
            fps,
            config: { damping: 14, stiffness: 120 },
          });
          return (
            <div
              key={i}
              style={{
                flex: 1,
                backgroundColor: COLORS.bgLight,
                borderRadius: 16,
                padding: "32px 28px",
                border: `1px solid ${step.color}33`,
                opacity: s,
                transform: `translateY(${interpolate(s, [0, 1], [40, 0])}px)`,
              }}
            >
              <div
                style={{
                  width: 48,
                  height: 48,
                  borderRadius: 24,
                  backgroundColor: `${step.color}22`,
                  border: `2px solid ${step.color}`,
                  display: "flex",
                  justifyContent: "center",
                  alignItems: "center",
                  fontFamily: FONT.heading,
                  fontSize: 24,
                  fontWeight: 800,
                  color: step.color,
                  marginBottom: 18,
                }}
              >
                {step.num}
              </div>
              <div
                style={{
                  fontFamily: FONT.heading,
                  fontSize: 24,
                  fontWeight: 700,
                  color: COLORS.white,
                  marginBottom: 10,
                }}
              >
                {step.title}
              </div>
              <div
                style={{
                  fontFamily: FONT.body,
                  fontSize: 18,
                  color: COLORS.gray,
                  lineHeight: 1.5,
                }}
              >
                {step.desc}
              </div>
            </div>
          );
        })}
      </div>

      {/* Mini simulation: bullet stops at wall */}
      <div
        style={{
          position: "relative",
          width: 1200,
          height: 200,
          marginTop: 50,
          backgroundColor: COLORS.bgLight,
          borderRadius: 12,
          border: `1px solid ${COLORS.grayDark}`,
          overflow: "hidden",
        }}
      >
        <div
          style={{
            position: "absolute",
            top: 12,
            left: 20,
            fontFamily: FONT.mono,
            fontSize: 16,
            color: COLORS.gray,
          }}
        >
          velox CCD @ 2 km/s
        </div>

        {/* Wall */}
        <div
          style={{
            position: "absolute",
            left: 900,
            top: 50,
            width: 24,
            height: 120,
            backgroundColor: COLORS.grayDark,
            borderRadius: 3,
            border: `1px solid ${COLORS.gray}`,
          }}
        />

        {/* Bullet */}
        <div
          style={{
            position: "absolute",
            left: bulletX,
            top: 100,
            width: 22,
            height: 9,
            borderRadius: 4,
            backgroundColor: COLORS.cyan,
            boxShadow: `0 0 12px ${COLORS.cyan}88`,
          }}
        />

        {/* Impact flash */}
        {flashOpacity > 0 && (
          <div
            style={{
              position: "absolute",
              left: 880,
              top: 70,
              width: 60,
              height: 60,
              borderRadius: 30,
              backgroundColor: `${COLORS.cyan}`,
              opacity: flashOpacity,
              filter: "blur(15px)",
            }}
          />
        )}

        {/* Green check */}
        <div
          style={{
            position: "absolute",
            right: 40,
            top: 60,
            fontSize: 72,
            color: COLORS.green,
            opacity: checkOpacity,
            textShadow: `0 0 30px ${COLORS.green}66`,
          }}
        >
          ✓
        </div>
      </div>
    </AbsoluteFill>
  );
};
