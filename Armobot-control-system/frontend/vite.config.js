import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      '/pico': {
        target: 'http://192.168.137.50',
        changeOrigin: true,
        rewrite: (path) => path.replace(/^\/pico/, '')
      }
    }
  }
})
