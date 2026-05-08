import fs from 'node:fs'
import path from 'node:path'

const docsDir = path.resolve(__dirname, '..')

/** 从 md 文件提取标题：优先 frontmatter title，否则第一个 # 标题 */
function getTitle(filePath: string): string {
  const content = fs.readFileSync(filePath, 'utf-8')
  const fmMatch = content.match(/^---\s*\n[\s\S]*?title:\s*(.+)\n[\s\S]*?---/)
  if (fmMatch) return fmMatch[1].trim().replace(/^["']|["']$/g, '')
  const h1Match = content.match(/^#\s+(.+)/m)
  if (h1Match) return h1Match[1].trim()
  return path.basename(filePath, '.md')
}

/** 从 md 文件提取 frontmatter order 字段 */
function getOrder(filePath: string): number {
  const content = fs.readFileSync(filePath, 'utf-8')
  const match = content.match(/^---\s*\n[\s\S]*?order:\s*(\d+)\n[\s\S]*?---/)
  return match ? parseInt(match[1]) : 999
}

/** 检查文件是否声明 sidebar: false */
function hasSidebarFalse(filePath: string): boolean {
  const content = fs.readFileSync(filePath, 'utf-8')
  return /^---\s*\n[\s\S]*?sidebar:\s*false\n[\s\S]*?---/.test(content)
}

interface SidebarItem {
  text: string
  link?: string
  collapsed?: boolean
  items?: SidebarItem[]
}

/** 扫描目录中的 md 文件（非 index），返回 sidebar items */
function scanDir(dirPath: string, urlPrefix: string): SidebarItem[] {
  if (!fs.existsSync(dirPath)) return []
  const entries = fs.readdirSync(dirPath, { withFileTypes: true })

  const mdFiles = entries
    .filter(e => e.isFile() && e.name.endsWith('.md') && e.name !== 'index.md')
    .map(e => ({
      name: e.name,
      path: path.join(dirPath, e.name),
      slug: e.name.replace('.md', ''),
    }))
    .sort((a, b) => getOrder(a.path) - getOrder(b.path) || a.name.localeCompare(b.name))

  return mdFiles.map(f => ({ text: getTitle(f.path), link: `${urlPrefix}${f.slug}` }))
}

/** 获取子目录的分组名：从 index.md 读取 title，否则用目录名 */
function getGroupName(dirPath: string): string {
  const indexPath = path.join(dirPath, 'index.md')
  if (fs.existsSync(indexPath)) return getTitle(indexPath)
  return path.basename(dirPath)
}

/** 获取子目录的排序值：从 index.md 读取 order */
function getGroupOrder(dirPath: string): number {
  const indexPath = path.join(dirPath, 'index.md')
  if (fs.existsSync(indexPath)) return getOrder(indexPath)
  return 999
}

/** 为一个 section 生成完整 sidebar */
function generateSectionSidebar(sectionDir: string, urlPrefix: string): SidebarItem[] {
  if (!fs.existsSync(sectionDir)) return []
  const result: SidebarItem[] = []
  const entries = fs.readdirSync(sectionDir, { withFileTypes: true })

  // index.md 作为第一个条目（除非声明 sidebar: false）
  const indexPath = path.join(sectionDir, 'index.md')
  if (fs.existsSync(indexPath) && !hasSidebarFalse(indexPath)) {
    result.push({ text: getTitle(indexPath), link: urlPrefix })
  }

  // 当前目录的 md 文件
  result.push(...scanDir(sectionDir, urlPrefix))

  // 子目录作为分组，按 order 排序
  const subDirs = entries
    .filter(e => e.isDirectory() && !e.name.startsWith('.'))
    .map(e => ({ name: e.name, path: path.join(sectionDir, e.name) }))
    .sort((a, b) => getGroupOrder(a.path) - getGroupOrder(b.path) || a.name.localeCompare(b.name))

  for (const dir of subDirs) {
    const subUrl = `${urlPrefix}${dir.name}/`
    const subItems = scanDir(dir.path, subUrl)
    const subIndexPath = path.join(dir.path, 'index.md')

    // 子目录有子文件：作为可折叠分组
    if (subItems.length > 0) {
      result.push({
        text: getGroupName(dir.path),
        collapsed: subItems.length > 8,
        items: subItems,
      })
    } else if (fs.existsSync(subIndexPath) && !hasSidebarFalse(subIndexPath)) {
      // 子目录只有 index.md：作为单链接
      result.push({ text: getTitle(subIndexPath), link: subUrl })
    }
  }

  return result
}

/** 生成完整 sidebar 配置 */
export function generateSidebar(): Record<string, SidebarItem[]> {
  const sidebar: Record<string, SidebarItem[]> = {}

  sidebar['/guide/'] = generateSectionSidebar(path.join(docsDir, 'guide'), '/guide/')
  sidebar['/cyber/'] = generateSectionSidebar(path.join(docsDir, 'cyber'), '/cyber/')
  sidebar['/reference/'] = generateSectionSidebar(path.join(docsDir, 'reference'), '/reference/')

  // modules - 每个有内容的模块获得独立 sidebar
  const modulesDir = path.join(docsDir, 'modules')
  if (!fs.existsSync(modulesDir)) return sidebar

  const allModuleLinks: SidebarItem[] = []
  const moduleDirs = fs.readdirSync(modulesDir, { withFileTypes: true })
    .filter(e => e.isDirectory())
    .sort((a, b) => a.name.localeCompare(b.name))

  for (const mod of moduleDirs) {
    const modPath = path.join(modulesDir, mod.name)
    const modUrl = `/modules/${mod.name}/`
    const indexPath = path.join(modPath, 'index.md')

    if (fs.existsSync(indexPath)) {
      allModuleLinks.push({ text: getTitle(indexPath), link: modUrl })
    }

    const modEntries = fs.readdirSync(modPath, { withFileTypes: true })
    const hasSubDirs = modEntries.some(e => e.isDirectory() && !e.name.startsWith('.'))
    const mdCount = modEntries.filter(e => e.isFile() && e.name.endsWith('.md') && e.name !== 'index.md').length

    if (hasSubDirs || mdCount > 0) {
      sidebar[`/modules/${mod.name}/`] = generateSectionSidebar(modPath, modUrl)
    }
  }

  sidebar['/modules/'] = [{ text: '全部模块', items: allModuleLinks }]
  return sidebar
}

/** 生成 nav 配置 */
export function generateNav() {
  return [
    { text: '指南', link: '/guide/', activeMatch: '/guide/' },
    { text: 'Cyber 框架', link: '/cyber/', activeMatch: '/cyber/' },
    {
      text: '核心链路',
      activeMatch: '/modules/(planning|control|perception|prediction|routing)/',
      items: [
        { text: 'Planning 规划', link: '/modules/planning/' },
        { text: 'Control 控制', link: '/modules/control/' },
        { text: 'Perception 感知', link: '/modules/perception/' },
        { text: 'Prediction 预测', link: '/modules/prediction/' },
        { text: 'Routing 路由', link: '/modules/routing/' },
      ],
    },
    {
      text: '基础设施',
      activeMatch: '/modules/(localization|map|drivers|canbus|transform)/',
      items: [
        { text: 'Localization 定位', link: '/modules/localization/' },
        { text: 'Map 地图', link: '/modules/map/' },
        { text: 'Drivers 驱动', link: '/modules/drivers/' },
        { text: 'Canbus 总线', link: '/modules/canbus/' },
        { text: 'Transform 坐标变换', link: '/modules/transform/' },
      ],
    },
    {
      text: '平台服务',
      activeMatch: '/modules/(dreamview|monitor|guardian|external-command|task-manager)/',
      items: [
        { text: 'Dreamview 可视化', link: '/modules/dreamview/' },
        { text: 'Monitor 监控', link: '/modules/monitor/' },
        { text: 'Guardian 安全守护', link: '/modules/guardian/' },
        { text: 'External Command', link: '/modules/external-command/' },
        { text: 'Task Manager', link: '/modules/task-manager/' },
      ],
    },
    {
      text: '其他模块',
      activeMatch: '/modules/(audio|bridge|v2x|common|storytelling|data|statistics)/',
      items: [
        { text: 'Audio 音频', link: '/modules/audio/' },
        { text: 'Bridge 桥接', link: '/modules/bridge/' },
        { text: 'V2X 车路协同', link: '/modules/v2x/' },
        { text: 'Common 公共库', link: '/modules/common/' },
        { text: '全部模块', link: '/modules/' },
      ],
    },
  ]
}
