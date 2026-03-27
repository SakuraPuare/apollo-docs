import { defineConfig } from 'vitepress'

export default defineConfig({
  title: 'Apollo Docs',
  description: 'Apollo 自动驾驶平台技术文档',
  lang: 'zh-CN',
  lastUpdated: true,

  themeConfig: {
    nav: [
      { text: '指南', link: '/guide/introduction' },
      { text: 'Cyber 框架', link: '/cyber/' },
      { text: '模块', link: '/modules/perception/' },
    ],

    sidebar: {
      '/guide/': [
        {
          text: '指南',
          items: [
            { text: '简介', link: '/guide/introduction' },
            { text: '快速开始', link: '/guide/getting-started' },
            { text: '架构概览', link: '/guide/architecture' },
          ],
        },
      ],
      '/cyber/': [
        {
          text: 'Cyber 中间件框架',
          items: [
            { text: '概览', link: '/cyber/' },
            { text: 'Node', link: '/cyber/node' },
            { text: 'Component', link: '/cyber/component' },
            { text: 'Message', link: '/cyber/message' },
            { text: 'Transport', link: '/cyber/transport' },
            { text: 'Scheduler', link: '/cyber/scheduler' },
            { text: 'Service Discovery', link: '/cyber/service-discovery' },
            { text: 'Parameter', link: '/cyber/parameter' },
            { text: 'Record', link: '/cyber/record' },
            { text: 'Logger', link: '/cyber/logger' },
            { text: 'Timer', link: '/cyber/timer' },
            { text: 'Class Loader', link: '/cyber/class-loader' },
            { text: 'Mainboard', link: '/cyber/mainboard' },
          ],
        },
      ],
      '/modules/': [
        {
          text: '感知与决策',
          items: [
            { text: 'Perception 感知', link: '/modules/perception/' },
            { text: 'Prediction 预测', link: '/modules/prediction/' },
            { text: 'Planning 规划', link: '/modules/planning/' },
            { text: 'Control 控制', link: '/modules/control/' },
          ],
        },
        {
          text: '定位与导航',
          items: [
            { text: 'Localization 定位', link: '/modules/localization/' },
            { text: 'Routing 路由', link: '/modules/routing/' },
            { text: 'Map 地图', link: '/modules/map/' },
            { text: 'Transform 坐标变换', link: '/modules/transform/' },
          ],
        },
        {
          text: '硬件与通信',
          items: [
            { text: 'Drivers 驱动', link: '/modules/drivers/' },
            { text: 'Canbus 总线', link: '/modules/canbus/' },
            { text: 'V2X 车路协同', link: '/modules/v2x/' },
            { text: 'Bridge 桥接', link: '/modules/bridge/' },
          ],
        },
        {
          text: '可视化与监控',
          items: [
            { text: 'Dreamview 可视化', link: '/modules/dreamview/' },
            { text: 'Monitor 监控', link: '/modules/monitor/' },
            { text: 'Guardian 安全守护', link: '/modules/guardian/' },
          ],
        },
        {
          text: '工具与数据',
          items: [
            { text: 'Calibration 标定', link: '/modules/calibration/' },
            { text: 'Data 数据', link: '/modules/data/' },
            { text: 'Audio 音频', link: '/modules/audio/' },
            { text: 'Common 公共', link: '/modules/common/' },
            { text: 'Storytelling 日志', link: '/modules/storytelling/' },
            { text: 'Task Manager 任务管理', link: '/modules/task-manager/' },
            { text: 'External Command 外部命令', link: '/modules/external-command/' },
          ],
        },
      ],
    },

    socialLinks: [
      { icon: 'github', link: 'https://github.com/ApolloAuto/apollo' },
    ],

    search: { provider: 'local' },
    outline: { level: [2, 3] },
  },
})
