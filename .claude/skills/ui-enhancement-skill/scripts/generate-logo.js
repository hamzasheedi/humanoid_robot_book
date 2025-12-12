const fs = require('fs');
const path = require('path');

/**
 * Generates SVG logos for light and dark modes based on the
 * Physical AI & Humanoid Robotics Textbook theme
 */
function generateLogo(frontendDir = './frontend') {
  const imgDir = path.join(frontendDir, 'static', 'img');

  // Create the img directory if it doesn't exist
  if (!fs.existsSync(imgDir)) {
    fs.mkdirSync(imgDir, { recursive: true });
  }

  // Create light mode logo
  const lightLogo = `<?xml version="1.0" encoding="UTF-8"?>
<svg width="200" height="60" viewBox="0 0 200 60" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <linearGradient id="logoGradient" x1="0%" y1="0%" x2="100%" y2="100%">
      <stop offset="0%" stop-color="#667eea" />
      <stop offset="100%" stop-color="#764ba2" />
    </linearGradient>
  </defs>

  <!-- Robot icon -->
  <g transform="translate(15, 10)">
    <rect x="0" y="0" width="40" height="40" rx="8" fill="url(#logoGradient)" stroke="#333" stroke-width="1"/>
    <circle cx="10" cy="15" r="3" fill="#fff"/>
    <circle cx="30" cy="15" r="3" fill="#fff"/>
    <rect x="15" y="25" width="10" height="10" rx="2" fill="#333"/>
    <rect x="5" y="5" width="30" height="5" rx="2" fill="#fff" opacity="0.7"/>
  </g>

  <!-- Text -->
  <text x="70" y="35" font-family="Arial, sans-serif" font-size="16" font-weight="bold" fill="#2c3e50">
    Physical AI
  </text>
  <text x="70" y="52" font-family="Arial, sans-serif" font-size="14" fill="#7f8c8d">
    Humanoid Robotics
  </text>
</svg>`;

  // Create dark mode logo
  const darkLogo = `<?xml version="1.0" encoding="UTF-8"?>
<svg width="200" height="60" viewBox="0 0 200 60" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <linearGradient id="logoGradient" x1="0%" y1="0%" x2="100%" y2="100%">
      <stop offset="0%" stop-color="#667eea" />
      <stop offset="100%" stop-color="#764ba2" />
    </linearGradient>
  </defs>

  <!-- Robot icon -->
  <g transform="translate(15, 10)">
    <rect x="0" y="0" width="40" height="40" rx="8" fill="url(#logoGradient)" stroke="#ccc" stroke-width="1"/>
    <circle cx="10" cy="15" r="3" fill="#eee"/>
    <circle cx="30" cy="15" r="3" fill="#eee"/>
    <rect x="15" y="25" width="10" height="10" rx="2" fill="#ddd"/>
    <rect x="5" y="5" width="30" height="5" rx="2" fill="#fff" opacity="0.7"/>
  </g>

  <!-- Text -->
  <text x="70" y="35" font-family="Arial, sans-serif" font-size="16" font-weight="bold" fill="#ecf0f1">
    Physical AI
  </text>
  <text x="70" y="52" font-family="Arial, sans-serif" font-size="14" fill="#bdc3c7">
    Humanoid Robotics
  </text>
</svg>`;

  // Write the logo files
  const lightLogoPath = path.join(imgDir, 'logo-light.svg');
  const darkLogoPath = path.join(imgDir, 'logo-dark.svg');

  fs.writeFileSync(lightLogoPath, lightLogo);
  fs.writeFileSync(darkLogoPath, darkLogo);

  console.log('Logos generated successfully!');
  console.log(`Created ${lightLogoPath}`);
  console.log(`Created ${darkLogoPath}`);

  // Update docusaurus.config.js to use the new logos
  const configPath = path.join(frontendDir, 'docusaurus.config.js');
  if (fs.existsSync(configPath)) {
    let configContent = fs.readFileSync(configPath, 'utf8');

    // Update favicon if present
    if (configContent.includes('favicon:')) {
      configContent = configContent.replace(
        /(favicon:\s*['"][^'"]*['"])/,
        'favicon: \'img/logo-light.svg\''
      );
    } else {
      configContent = configContent.replace(
        /themeConfig:\s*{/i,
        `themeConfig: {
    image: 'img/logo-light.svg', // Default image for meta tags`
      );
    }

    // Update navbar logo configuration
    if (configContent.includes('navbar: {')) {
      // If navbar config exists, update it
      if (configContent.includes('logo: {')) {
        configContent = configContent.replace(
          /(logo:\s*{[^}]*})/,
          `logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo-light.svg',
        srcDark: 'img/logo-dark.svg',
      }`
        );
      } else {
        // Add logo config to navbar
        configContent = configContent.replace(
          /navbar:\s*{/,
          `navbar: {
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo-light.svg',
        srcDark: 'img/logo-dark.svg',
      },`
        );
      }
    } else {
      // Add navbar config with logo
      configContent = configContent.replace(
        /themeConfig:\s*{/,
        `navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo-light.svg',
        srcDark: 'img/logo-dark.svg',
      },
      items: [
        {to: '/docs/intro', label: 'Textbook', position: 'left'},
        {href: 'https://github.com/your-repo/humanoid-robot-book', label: 'GitHub', position: 'right'},
      ],
    },
    themeConfig: {`
      );
    }

    fs.writeFileSync(configPath, configContent);
    console.log('Updated docusaurus.config.js with new logo configuration');
  }
}

// Export the function
module.exports = { generateLogo };

// If running directly
if (require.main === module) {
  const frontendDir = process.argv[2] || './frontend';
  generateLogo(frontendDir);
}