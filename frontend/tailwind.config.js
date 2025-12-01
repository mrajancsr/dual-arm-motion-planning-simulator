/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        'dark-bg': '#1a1a1a',
        'dark-panel': '#2a2a2a',
        'left-arm': '#3b82f6',
        'right-arm': '#ef4444',
        'tree-node': '#ffffff',
        'solution-path': '#22c55e',
        'obstacle': '#f97316',
      }
    },
  },
  plugins: [],
}

