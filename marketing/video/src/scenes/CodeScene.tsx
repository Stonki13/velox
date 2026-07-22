import React from "react";
import {
  AbsoluteFill,
  useCurrentFrame,
  useVideoConfig,
  spring,
  interpolate,
} from "remotion";
import { COLORS, FONT } from "../theme";

interface Token {
  text: string;
  color: string;
}

const CODE_LINES: Token[][] = [
  [
    { text: "velox", color: COLORS.code.type },
    { text: "::", color: COLORS.code.plain },
    { text: "World", color: COLORS.code.type },
    { text: " world", color: COLORS.code.plain },
    { text: ";", color: COLORS.code.plain },
  ],
  [
    { text: "world", color: COLORS.code.plain },
    { text: ".", color: COLORS.code.plain },
    { text: "gravity", color: COLORS.code.func },
    { text: " = ", color: COLORS.code.keyword },
    { text: "{", color: COLORS.code.plain },
    { text: "0", color: COLORS.code.number },
    { text: ", ", color: COLORS.code.plain },
    { text: "-9.81f", color: COLORS.code.number },
    { text: ", ", color: COLORS.code.plain },
    { text: "0", color: COLORS.code.number },
    { text: "}", color: COLORS.code.plain },
    { text: ";", color: COLORS.code.plain },
  ],
  [],
  [
    { text: "auto", color: COLORS.code.keyword },
    { text: " ground ", color: COLORS.code.plain },
    { text: "= ", color: COLORS.code.keyword },
    { text: "world", color: COLORS.code.plain },
    { text: ".", color: COLORS.code.plain },
    { text: "addStaticPlane", color: COLORS.code.func },
    { text: "({", color: COLORS.code.plain },
    { text: "0", color: COLORS.code.number },
    { text: ", ", color: COLORS.code.plain },
    { text: "1", color: COLORS.code.number },
    { text: ", ", color: COLORS.code.plain },
    { text: "0", color: COLORS.code.number },
    { text: "}, ", color: COLORS.code.plain },
    { text: "0.0f", color: COLORS.code.number },
    { text: ");", color: COLORS.code.plain },
  ],
  [
    { text: "auto", color: COLORS.code.keyword },
    { text: " bullet ", color: COLORS.code.plain },
    { text: "= ", color: COLORS.code.keyword },
    { text: "world", color: COLORS.code.plain },
    { text: ".", color: COLORS.code.plain },
    { text: "addSphere", color: COLORS.code.func },
    { text: "({", color: COLORS.code.plain },
    { text: "0", color: COLORS.code.number },
    { text: ", ", color: COLORS.code.plain },
    { text: "1", color: COLORS.code.number },
    { text: ", ", color: COLORS.code.plain },
    { text: "0", color: COLORS.code.number },
    { text: "}, ", color: COLORS.code.plain },
    { text: "0.05f", color: COLORS.code.number },
    { text: ", ", color: COLORS.code.plain },
    { text: "0.01f", color: COLORS.code.number },
    { text: ");", color: COLORS.code.plain },
  ],
  [
    { text: "world", color: COLORS.code.plain },
    { text: ".", color: COLORS.code.plain },
    { text: "body", color: COLORS.code.func },
    { text: "(bullet)", color: COLORS.code.plain },
    { text: ".", color: COLORS.code.plain },
    { text: "velocity", color: COLORS.code.func },
    { text: " = ", color: COLORS.code.keyword },
    { text: "{", color: COLORS.code.plain },
    { text: "0", color: COLORS.code.number },
    { text: ", ", color: COLORS.code.plain },
    { text: "-2000", color: COLORS.code.number },
    { text: ", ", color: COLORS.code.plain },
    { text: "0", color: COLORS.code.number },
    { text: "}", color: COLORS.code.plain },
    { text: ";", color: COLORS.code.plain },
    { text: "   // 2 km/s", color: COLORS.code.comment },
  ],
  [],
  [
    { text: "world", color: COLORS.code.plain },
    { text: ".", color: COLORS.code.plain },
    { text: "step", color: COLORS.code.func },
    { text: "(", color: COLORS.code.plain },
    { text: "1.0f", color: COLORS.code.number },
    { text: " / ", color: COLORS.code.plain },
    { text: "60.0f", color: COLORS.code.number },
    { text: ");", color: COLORS.code.plain },
    { text: "   // CCD catches it. No tunneling.", color: COLORS.code.comment },
  ],
];

export const CodeScene: React.FC = () => {
  const frame = useCurrentFrame();
  const { fps } = useVideoConfig();

  const headerSpring = spring({ frame, fps, config: { damping: 15 } });

  const CHARS_PER_FRAME = 2.5;
  const START_FRAME = 20;

  // Flatten all tokens to count characters
  let totalChars = 0;
  const lineCharCounts = CODE_LINES.map((line) => {
    const count = line.reduce((sum, t) => sum + t.text.length, 0);
    totalChars += count + 1; // +1 for newline
    return count;
  });

  const elapsedChars = Math.max(0, (frame - START_FRAME) * CHARS_PER_FRAME);

  let charBudget = elapsedChars;

  // Cursor blink
  const cursorVisible = Math.floor(frame / 8) % 2 === 0;

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
          fontSize: 52,
          fontWeight: 800,
          color: COLORS.white,
          letterSpacing: 4,
          opacity: headerSpring,
        }}
      >
        GET STARTED IN SECONDS
      </div>

      {/* Code editor window */}
      <div
        style={{
          width: 1300,
          marginTop: 50,
          backgroundColor: "#1A1A2E",
          borderRadius: 16,
          border: `1px solid ${COLORS.grayDark}`,
          overflow: "hidden",
          opacity: interpolate(frame, [10, 20], [0, 1], {
            extrapolateLeft: "clamp",
            extrapolateRight: "clamp",
          }),
        }}
      >
        {/* Title bar */}
        <div
          style={{
            display: "flex",
            alignItems: "center",
            gap: 8,
            padding: "14px 20px",
            backgroundColor: "#16162A",
            borderBottom: `1px solid ${COLORS.grayDark}`,
          }}
        >
          <div style={{ width: 14, height: 14, borderRadius: 7, backgroundColor: "#FF5F57" }} />
          <div style={{ width: 14, height: 14, borderRadius: 7, backgroundColor: "#FEBC2E" }} />
          <div style={{ width: 14, height: 14, borderRadius: 7, backgroundColor: "#28C840" }} />
          <span
            style={{
              fontFamily: FONT.mono,
              fontSize: 16,
              color: COLORS.gray,
              marginLeft: 12,
            }}
          >
            main.cpp
          </span>
        </div>

        {/* Code body */}
        <div style={{ padding: "28px 32px", minHeight: 380 }}>
          {CODE_LINES.map((line, li) => {
            const lineStartChar = CODE_LINES.slice(0, li).reduce(
              (sum, l) => sum + l.reduce((s, t) => s + t.text.length, 0) + 1,
              0
            );

            let remainingBudget = charBudget - lineStartChar;
            if (remainingBudget <= 0 && li > 0) {
              return <div key={li} style={{ height: 36 }} />;
            }

            let lineCharsShown = Math.min(
              Math.max(0, remainingBudget),
              lineCharCounts[li]
            );

            // Render tokens up to budget
            let charsUsed = 0;
            const visibleTokens: React.ReactNode[] = [];

            for (let ti = 0; ti < line.length; ti++) {
              const token = line[ti];
              const charsLeft = lineCharsShown - charsUsed;
              if (charsLeft <= 0) break;

              const visibleText = token.text.slice(0, charsLeft);
              visibleTokens.push(
                <span key={ti} style={{ color: token.color }}>
                  {visibleText}
                </span>
              );
              charsUsed += token.text.length;
            }

            const isCurrentLine =
              remainingBudget > 0 && remainingBudget < lineCharCounts[li] + 1;

            return (
              <div
                key={li}
                style={{
                  fontFamily: FONT.mono,
                  fontSize: 26,
                  lineHeight: "36px",
                  whiteSpace: "pre",
                  minHeight: 36,
                }}
              >
                <span style={{ color: COLORS.grayDark, marginRight: 24, userSelect: "none" }}>
                  {String(li + 1).padStart(2, " ")}
                </span>
                {visibleTokens}
                {isCurrentLine && cursorVisible && (
                  <span
                    style={{
                      display: "inline-block",
                      width: 2,
                      height: 26,
                      backgroundColor: COLORS.cyan,
                      marginLeft: 1,
                      verticalAlign: "text-bottom",
                    }}
                  />
                )}
              </div>
            );
          })}
        </div>
      </div>

      {/* Bottom note */}
      <div
        style={{
          fontFamily: FONT.body,
          fontSize: 22,
          color: COLORS.gray,
          marginTop: 30,
          opacity: interpolate(frame, [250, 265], [0, 1], {
            extrapolateLeft: "clamp",
            extrapolateRight: "clamp",
          }),
        }}
      >
        cmake -B build && cmake --build build — that's it
      </div>
    </AbsoluteFill>
  );
};
