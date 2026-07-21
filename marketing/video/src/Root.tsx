import React from "react";
import { Composition } from "remotion";
import { VeloxVideo } from "./VeloxVideo";

export const FPS = 30;
export const WIDTH = 1920;
export const HEIGHT = 1080;
export const DURATION = 1800; // 60 seconds

export const Root: React.FC = () => {
  return (
    <Composition
      id="VeloxPromo"
      component={VeloxVideo}
      durationInFrames={DURATION}
      fps={FPS}
      width={WIDTH}
      height={HEIGHT}
    />
  );
};
