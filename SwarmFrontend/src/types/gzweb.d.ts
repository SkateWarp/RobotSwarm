declare module 'gzweb' {
  export class Asset { }
  export interface AssetViewerConfig { }
  export class AssetViewer {
    constructor(config?: AssetViewerConfig);
  }
  export class AudioTopic { }
  export class Color { }
  export class FuelServer { }
  export class Gamepad { }
  export class Inertia { }
  export class Material { }
  export class ModelUserData { }
  export class PBRMaterial { }
  export class Pose { }
  export class Publisher {
    publish(message: any): void;
  }
  export class Scene { }
  export interface SceneManagerConfig {
    /**
     * ElementId is the id of the HTML element that will hold the rendering
     * context. If not specified, the id gz-scene will be used.
     */
    elementId?: string;

    /**
     * A websocket url that points to a Gazebo server.
     */
    websocketUrl?: string;

    /**
     * An authentication key for the websocket server.
     */
    websocketKey?: string;

    /**
     * The name of a an audio control topic, used to play audio files.
     */
    audioTopic?: string;

    /**
     * Name of the topic to advertise.
     */
    topicName?: string;

    /**
     * Message type of the topic to advertise.
     */
    msgType?: string;

    /**
     * Message data of the topic to advertise.
     */
    msgData?: any;

    /**
     * Whether or not lights in models are visible.
     */
    enableLights?: boolean;
  }
  export class SceneManager {
    constructor(config?: SceneManagerConfig);

    destroy(): void;
    getConnectionStatus(): string;
    getConnectionStatusAsObservable(): Observable<boolean>;
    resize(): void;
    snapshot(): void;
    resetView(): void;
    follow(entityName: string): void;
    thirdPersonFollow(entityName: string): void;
    firstPerson(entityName: string): void;
    moveTo(entityName: string): void;
    select(entityName: string): void;
    publish(): void;
    getModels(): any[];
    disconnect(): void;
    connect(url: string, key?: string): void;
    advertise(topic: string, msgTypeName: string): Publisher;
    subscribeToTopic(topic: Topic): void;
    unsubscribeFromTopic(name: string): void;
    play(): void;
    pause(): void;
    stop(): void;
  }
  export class SDFParser { }
  export class Topic {
    name: string;
    msgType: string;
    callback: (message: any) => void;
  }
  export class Transport {
    connect(url: string, key?: string): void;
    disconnect(): void;
  }

  // Global namespace
  const REVISION: string;
  const isTouchDevice: boolean;
} 