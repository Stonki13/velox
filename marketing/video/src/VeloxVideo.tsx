import React from "react";
import {
  AbsoluteFill,
  Sequence,
  useCurrentFrame,
  useVideoConfig,
  interpolate,
} from "remotion";
import { TitleScene } from "./scenes/TitleScene";
import { ProblemScene } from "./scenes/ProblemScene";
import { SolutionScene } from "./scenes/SolutionScene";
import { DemoScene } from "./scenes/DemoScene";
import { BenchmarkScene } from "./scenes/BenchmarkScene";
import { CodeScene } from "./scenes/CodeScene";
import { FeaturesScene } from "./scenes/FeaturesScene";
import { CTAScene } from "./scenes/CTAScene";
import { COLORS } from "./theme";

const FADE = 12; // crossfade frames

interface SceneDef {
  component: React.FC;
  from: number;
  duration: number;
}

const SCENES: SceneDef[] = [
  { component: TitleScene, from: 0, duration: 90 },
  { component: ProblemScene, from: 90, duration: 150 },
  { component: SolutionScene, from: 240, duration: 210 },
  { component: DemoScene, from: 450, duration: 300 },
  { component: BenchmarkScene, from: 750, duration: 300 },
  { component: CodeScene, from: 1050, duration: 300 },
  { component: FeaturesScene, from: 1350, duration: 240 },
  { component: CTAScene, from: 1590, duration: 210 },
];

const SceneWithFade: React.FC<{
  component: React.FC;
  duration: number;
}> = ({ component: Scene, duration }) => {
  const frame = useCurrentFrame();

  const fadeIn = interpolate(frame, [0, FADE], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
  });
  const fadeOut = interpolate(
    frame,
    [duration - FADE, duration],
    [1, 0],
    { extrapolateLeft: "clamp", extrapolateRight: "clamp" }
  );
  const opacity = Math.min(fadeIn, fadeOut);

  return (
    <AbsoluteFill style={{ opacity }}>
      <Scene />
    </AbsoluteFill>
  );
};

export const VeloxVideo: React.FC = () => {
  return (
    <AbsoluteFill style={{ backgroundColor: COLORS.bg }}>
      {SCENES.map((scene, i) => (
        <Sequence
          key={i}
          from={scene.from}
          durationInFrames={scene.duration}
        >
          <SceneWithFade
            component={scene.component}
            duration={scene.duration}
          />
        </Sequence>
      ))}
    </AbsoluteFill>
  );
};
