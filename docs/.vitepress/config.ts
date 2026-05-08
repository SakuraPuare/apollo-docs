import { defineConfig } from 'vitepress'
import { withMermaid } from 'vitepress-plugin-mermaid'
import lightbox from 'vitepress-plugin-lightbox'
import { groupIconMdPlugin, groupIconVitePlugin } from 'vitepress-plugin-group-icons'
import {
  GitChangelog,
  GitChangelogMarkdownSection,
} from '@nolebase/vitepress-plugin-git-changelog/vite'
import { generateSidebar, generateNav } from './sidebar'
import { frontmatterGuard } from './frontmatter-guard'

/** GitHub Pages: https://<user>.github.io/<repo>/ */
const base = '/apollo-docs/'
/** 本站文档源代码（commit 链接、协作入口） */
const docsRepoURL = 'https://github.com/ShuYingJiYu/apollo-docs'
/** 本站仓库主账号，用于 Git 变更日志里「贡献者」头像与主页链接（非 GitHub noreply 邮箱时需 mapAuthors） */
const docsGitHubOwner = 'SakuraPuare'
/** Apollo 官方上游，文档内容参考的源码与发布仓库 */
const apolloUpstreamURL = 'https://github.com/ApolloAuto/apollo'


export default withMermaid(defineConfig({
  base,
  title: 'Apollo Docs',
  description: 'Apollo 自动驾驶平台技术文档',
  lang: 'zh-CN',
  lastUpdated: true,

  head: [
    ['link', { rel: 'icon', href: `${base}favicon.ico` }],
  ],

  markdown: {
    languageAlias: {
      starlark: 'python',
      bazel: 'python',
      bzl: 'python',
      dbc: 'ini',
    },
    config: (md) => {
      md.use(lightbox, {})
      md.use(groupIconMdPlugin)
    },
  },

  vite: {
    plugins: [
      frontmatterGuard(),
      groupIconVitePlugin(),
      GitChangelog({
        repoURL: () => docsRepoURL,
        mapAuthors: [
          {
            name: 'Steven Moder',
            username: docsGitHubOwner,
            mapByEmailAliases: ['java20131114@gmail.com'],
          },
        ],
      }),
      GitChangelogMarkdownSection(),
    ],
    optimizeDeps: {
      exclude: [
        '@nolebase/vitepress-plugin-enhanced-readabilities/client',
        '@nolebase/vitepress-plugin-git-changelog/client',
      ],
    },
    ssr: {
      noExternal: [
        '@nolebase/vitepress-plugin-enhanced-readabilities',
        '@nolebase/vitepress-plugin-git-changelog',
      ],
    },
  },

  themeConfig: {
    logo: '/logo.png',
    siteTitle: 'Docs',

    nav: generateNav(),

    sidebar: generateSidebar(),

    socialLinks: [
      {
        icon: 'github',
        link: docsRepoURL,
        ariaLabel: '本站文档源代码仓库（apollo-docs）',
      },
      {
        icon: 'github',
        link: apolloUpstreamURL,
        ariaLabel: 'Apollo 上游参考仓库（ApolloAuto/apollo）',
      },
    ],

    footer: {
      message: [
        `<span class="apollo-footer-byline">`,
        `Apollo 技术导读由社区整理，内容参考 `,
        `<a href="${apolloUpstreamURL}" target="_blank" rel="noopener noreferrer">ApolloAuto/apollo</a> `,
        `开源仓库；与百度及 Apollo 商业产品无隶属关系。<br />`,
        `维护与协作见 `,
        `<a href="${docsRepoURL}" target="_blank" rel="noopener noreferrer">${docsGitHubOwner}/apollo-docs</a>。`,
        `</span>`,
      ].join(''),
      copyright:
        `© ${new Date().getFullYear()} Apollo Docs 读者共建 · `
        + `<a href="${docsRepoURL}/graphs/contributors" target="_blank" rel="noopener noreferrer">致谢贡献者</a>`,
    },

    search: { provider: 'local' },
    outline: { level: [2, 3] },
  },
}))
