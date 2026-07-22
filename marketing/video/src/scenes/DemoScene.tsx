import React from "react";
import {
  AbsoluteFill,
  useCurrentFrame,
  useVideoConfig,
  spring,
  interpolate,
} from "remotion";
import { COLORS, FONT } from "../theme";

interface Ball {
  x: number;
  y: number;
  vx: number;
  vy: number;
  r: number;
  color: string;
}

const GROUND_Y = 520;
const GRAVITY = 0.35;
const RESTITUTION = 0.65;
const WALL_LEFT = 60;
const WALL_RIGHT = 1540;

function simulateBall(
  ball: Ball,
  totalFrames: number
): { x: number; y: number } {
  let { x, y, vx, vy, r } = ball;
  for (let f = 0; f < totalFrames; f++) {
    vy += GRAVITY;
    x += vx;
    y += vy;

    if (y + r > GROUND_Y) {
      y = GROUND_Y - r;
      vy = -vy * RESTITUTION;
      vx *= 0.98;
    }
    if (x - r < WALL_LEFT) {
      x = WALL_LEFT + r;
      vx = -vx * RESTITUTION;
    }
    if (x + r > WALL_RIGHT) {
      x = WALL_RIGHT - r;
      vx = -vx * RESTITUTION;
    }
  }
  return { x, y };
}

const BALLS: Ball[] = [
  { x: 300, y: 80, vx: 3.2, vy: 0, r: 22, color: COLORS.cyan },
  { x: 500, y: 40, vx: -1.5, vy: 1, r: 18, color: "#FF88CC" },
  { x: 700, y: 100, vx: 2.0, vy: -2, r: 26, color: COLORS.orange },
  { x: 900, y: 30, vx: -2.8, vy: 0, r: 16, color: COLORS.green },
  { x: 1100, y: 70, vx: 1.0, vy: 2, r: 20, color: "#AA88FF" },
  { x: 400, y: 150, vx: 4.0, vy: -1, r: 14, color: "#FFDD44" },
  { x: 800, y: 60, vx: -3.5, vy: 0, r: 24, color: "#44DDFF" },
  { x: 1200, y: 120, vx: 0.5, vy: -3, r: 19, color: "#FF6688" },
];

// Fast bullet that hits the right wall and bounces
const BULLET_START_X = 100;
const BULLET_Y = 300;
const BULLET_SPEED = 18;
const BULLET_WALL_X = 1400;

export const DemoScene: React.FC = () => {
  const frame = useCurrentFrame();
  const { fps } = useVideoConfig();

  const headerSpring = spring({ frame, fps, config: { damping: 15 } });

  // Bullet position
  let bulletX = BULLET_START_X + frame * BULLET_SPEED;
  let bulletBounced = false;
  if (bulletX > BULLET_WALL_X) {
    bulletX = BULLET_WALL_X - (bulletX - BULLET_WALL_X);
    bulletBounced = true;
  }

  const impactFrame = Math.floor(
    (BULLET_WALL_X - BULLET_START_X) / BULLET_SPEED
  );
  const flashOpacity = interpolate(
    frame,
    [impactFrame, impactFrame + 3, impactFrame + 12],
    [0, 0.9, 0],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );

  return (
    <AbsoluteFill
      style={{
        backgroundColor: COLORS.bg,
        justifyContent: "flex-start",
        alignItems: "center",
        paddingTop: 60,
      }}
    >
      <div
        style={{
          fontFamily: FONT.heading,
          fontSize: 52,
          fontWeight: 800,
          color: COLORS.white,
          letterSpacing: 4,
          opacity: headerSpring,
        }}
      >
        SEE IT IN ACTION
      </div>
      <div
        style={{
          fontFamily: FONT.body,
          fontSize: 22,
          color: COLORS.gray,
          marginTop: 6,
          opacity: interpolate(frame, [8, 20], [0, 1], {
            extrapolateLeft: "clamp",
            extrapolateRight: "clamp",
          }),
        }}
      >
        2 km/s projectile · 8 dynamic bodies · zero tunneling
      </div>

      {/* Simulation viewport */}
      <div
        style={{
          position: "relative",
          width: 1600,
          height: 580,
          marginTop: 40,
          backgroundColor: COLORS.bgLight,
          borderRadius: 16,
          border: `1px solid ${COLORS.grayDark}`,
          overflow: "hidden",
        }}
      >
        {/* Grid lines */}
        {Array.from({ length: 16 }).map((_, i) => (
          <div
            key={`v${i}`}
            style={{
              position: "absolute",
              left: 100 * i,
              top: 0,
              width: 1,
              height: "100%",
              backgroundColor: `${COLORS.grayDark}33`,
            }}
          />
        ))}
        {Array.from({ length: 6 }).map((_, i) => (
          <div
            key={`h${i}`}
            style={{
              position: "absolute",
              top: 100 * i,
              left: 0,
              width: "100%",
              height: 1,
              backgroundColor: `${COLORS.grayDark}33`,
            }}
          />
        ))}

        {/* Ground */}
        <div
          style={{
            position: "absolute",
            left: 0,
            top: GROUND_Y,
            width: "100%",
            height: 4,
            backgroundColor: COLORS.grayDark,
          }}
        />
        <div
          style={{
            position: "absolute",
            left: 0,
            top: GROUND_Y + 4,
            width: "100%",
            height: 60,
            backgroundColor: `${COLORS.grayDark}22`,
          }}
        />

        {/* Right wall (bullet target) */}
        <div
          style={{
            position: "absolute",
            left: BULLET_WALL_X,
            top: 200,
            width: 20,
            height: 250,
            backgroundColor: COLORS.grayDark,
            borderRadius: 3,
            border: `1px solid ${COLORS.gray}`,
          }}
        />

        {/* Bouncing balls */}
        {BALLS.map((ball, i) => {
          const delay = 10 + i * 5;
          const localFrame = Math.max(0, frame - delay);
          const pos = simulateBall(ball, localFrame);
          const ballOpacity = interpolate(frame, [delay, delay + 8], [0, 1], {
            extrapolateLeft: "clamp",
            extrapolateRight: "clamp",
          });
          return (
            <div
              key={i}
              style={{
                position: "absolute",
                left: pos.x - ball.r,
                top: pos.y - ball.r,
                width: ball.r * 2,
                height: ball.r * 2,
                borderRadius: "50%",
                backgroundColor: ball.color,
                opacity: ballOpacity,
                boxShadow: `0 0 12px ${ball.color}66`,
              }}
            />
          );
        })}

        {/* Bullet */}
        <div
          style={{
            position: "absolute",
            left: bulletX,
            top: BULLET_Y,
            width: 30,
            height: 8,
            borderRadius: 4,
            backgroundColor: COLORS.orange,
            boxShadow: `0 0 16px ${COLORS.orange}AA`,
            transform: bulletBounced ? "scaleX(-1)" : "none",
          }}
        />

        {/* Bullet trail */}
        {[1, 2, 3, 4].map((i) => {
          const trailX = bulletBounced
            ? bulletX + i * 25
            : bulletX - i * 25;
          return (
            <div
              key={i}
              style={{
                position: "absolute",
                left: trailX,
                top: BULLET_Y + 1,
                width: 18,
                height: 5,
                borderRadius: 3,
                backgroundColor: `${COLORS.orange}${(40 - i * 8).toString(16).padStart(2, "0")}`,
              }}
            />
          );
        })}

        {/* Impact flash */}
        {flashOpacity > 0 && (
          <div
            style={{
              position: "absolute",
              left: BULLET_WALL_X - 30,
              top: BULLET_Y - 25,
              width: 60,
              height: 60,
              borderRadius: 30,
              backgroundColor: COLORS.orange,
              opacity: flashOpacity,
              filter: "blur(12px)",
            }}
          />
        )}

        {/* HUD labels */}
        <div
          style={{
            position: "absolute",
            bottom: 16,
            left: 24,
            fontFamily: FONT.mono,
            fontSize: 16,
            color: COLORS.gray,
          }}
        >
          velox::World · 60 Hz · PCS enabled
        </div>
        <div
          style={{
            position: "absolute",
            bottom: 16,
            right: 24,
            fontFamily: FONT.mono,
            fontSize: 16,
            color: COLORS.green,
          }}
        >
          0 tunneling events
        </div>
      </div>
    </AbsoluteFill>
  );
};
