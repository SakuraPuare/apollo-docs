import fs from 'node:fs'
import path from 'node:path'
import type { Plugin } from 'vite'

const docsDir = path.resolve(__dirname, '..')

export function frontmatterGuard(): Plugin {
  return {
    name: 'frontmatter-guard',
    buildStart() {
      const errors: string[] = []
      checkDir(docsDir, errors)
      if (errors.length) {
        throw new Error(
          `Frontmatter validation failed:\n${errors.join('\n')}\n\n`
          + 'Fix: add "title: ..." to frontmatter, or add "noTitle: true" to opt out.',
        )
      }
    },
  }
}

function checkDir(dir: string, errors: string[]) {
  for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
    if (entry.name.startsWith('.')) continue
    const full = path.join(dir, entry.name)
    if (entry.isDirectory()) {
      checkDir(full, errors)
    } else if (entry.name.endsWith('.md')) {
      const content = fs.readFileSync(full, 'utf-8')
      if (!hasFrontmatter(content)) {
        errors.push(`  - ${path.relative(docsDir, full)}: missing frontmatter title`)
      }
    }
  }
}

function hasFrontmatter(content: string): boolean {
  const match = content.match(/^---\s*\n([\s\S]*?)\n---/)
  if (!match) return false
  const fm = match[1]
  if (/^title:\s*.+/m.test(fm)) return true
  if (/^noTitle:\s*true/m.test(fm)) return true
  return false
}
