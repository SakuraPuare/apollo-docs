<script setup>
import DefaultTheme from 'vitepress/theme'
import { onMounted } from 'vue'
import { useRouter } from 'vitepress'
import mediumZoom from 'medium-zoom'

const { Layout } = DefaultTheme
const router = useRouter()

const setupMediumZoom = () => {
  mediumZoom('[data-zoomable]', {
    background: 'transparent',
  })
}

onMounted(setupMediumZoom)
router.onAfterRouteChange = setupMediumZoom
</script>

<template>
  <Layout />
</template>

<style>
.medium-zoom-overlay {
  backdrop-filter: blur(5rem);
}

.medium-zoom-overlay,
.medium-zoom-image--opened {
  z-index: 999;
}

/* 首页 Hero logo 下方柔光（径向渐变 + 模糊） */
.VPHome .VPHero.has-image .image-container::before {
  content: '';
  position: absolute;
  left: 50%;
  bottom: 10%;
  width: min(92%, 300px);
  height: 26%;
  transform: translateX(-50%);
  border-radius: 50%;
  background: radial-gradient(
    ellipse 100% 100% at 50% 35%,
    color-mix(in srgb, var(--vp-c-brand-1) 55%, transparent) 0%,
    color-mix(in srgb, var(--vp-c-brand-2) 22%, transparent) 45%,
    transparent 70%
  );
  filter: blur(20px);
  opacity: 0.82;
  pointer-events: none;
  z-index: 0;
  animation: apollo-home-logo-glow 6s ease-in-out infinite alternate;
}

.dark .VPHome .VPHero.has-image .image-container::before {
  background: radial-gradient(
    ellipse 100% 100% at 50% 35%,
    color-mix(in srgb, var(--vp-c-brand-1) 65%, transparent) 0%,
    color-mix(in srgb, var(--vp-c-brand-2) 32%, transparent) 48%,
    transparent 72%
  );
}

@keyframes apollo-home-logo-glow {
  from {
    opacity: 0.55;
    transform: translateX(-50%) scale(0.94);
  }
  to {
    opacity: 0.92;
    transform: translateX(-50%) scale(1.06);
  }
}

@media (prefers-reduced-motion: reduce) {
  .VPHome .VPHero.has-image .image-container::before {
    animation: none;
    opacity: 0.75;
    transform: translateX(-50%) scale(1);
  }
}

.VPHome .VPHero.has-image .image-container .image-bg {
  z-index: 0;
}

.VPHome .VPHero.has-image .image-container .image-src {
  position: absolute;
  z-index: 1;
}
</style>
