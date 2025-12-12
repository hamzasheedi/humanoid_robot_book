const { enhanceHomepage } = require('./enhance-homepage');
const { createNavbar } = require('./create-navbar');
const { fixDarkMode } = require('./fix-dark-mode');
const { generateLogo } = require('./generate-logo');
const { enhanceChatbot } = require('./enhance-chatbot');
const { generateCSS } = require('./generate-css');
const { createComponents } = require('./create-components');

/**
 * Main UI Enhancement Skill Implementation
 *
 * This skill enhances the UI of the Physical AI & Humanoid Robotics Textbook
 * by implementing all the requested UI improvements.
 */
class UIEnhancementSkill {
  constructor(frontendDir = './frontend') {
    this.frontendDir = frontendDir;
  }

  /**
   * Execute all UI enhancement tasks
   */
  async executeAll() {
    console.log('🚀 Starting UI Enhancement Skill...');

    try {
      // 1. Generate new logos
      console.log('🎨 Generating new logos...');
      generateLogo(this.frontendDir);

      // 2. Enhance homepage
      console.log('🏠 Enhancing homepage...');
      enhanceHomepage(this.frontendDir);

      // 3. Create new navbar
      console.log('🧭 Creating responsive navbar...');
      createNavbar(this.frontendDir);

      // 4. Fix dark mode issues
      console.log('💡 Fixing dark mode problems...');
      fixDarkMode(this.frontendDir);

      // 5. Enhance chatbot window
      console.log('💬 Enhancing chatbot window...');
      enhanceChatbot(this.frontendDir);

      // 6. Generate modular CSS
      console.log('🧩 Generating modular CSS...');
      generateCSS(this.frontendDir);

      // 7. Create React components
      console.log('⚙️ Creating React components...');
      createComponents(this.frontendDir);

      console.log('✅ UI Enhancement Skill completed successfully!');
      console.log('📋 Summary of changes made:');
      console.log('   - Created new light/dark mode logos');
      console.log('   - Enhanced homepage with modern layout');
      console.log('   - Created responsive navbar with auth elements');
      console.log('   - Fixed dark mode contrast and visibility issues');
      console.log('   - Enhanced chatbot with better UI and animations');
      console.log('   - Generated modular CSS for consistent styling');
      console.log('   - Created reusable React components');

    } catch (error) {
      console.error('❌ Error during UI Enhancement:', error);
      throw error;
    }
  }

  /**
   * Execute specific UI enhancement tasks
   */
  async executeTasks(tasks = []) {
    console.log('🚀 Starting selected UI Enhancement tasks...');

    const taskMap = {
      'logo': () => generateLogo(this.frontendDir),
      'homepage': () => enhanceHomepage(this.frontendDir),
      'navbar': () => createNavbar(this.frontendDir),
      'dark-mode': () => fixDarkMode(this.frontendDir),
      'chatbot': () => enhanceChatbot(this.frontendDir),
      'css': () => generateCSS(this.frontendDir),
      'components': () => createComponents(this.frontendDir),
    };

    for (const task of tasks) {
      if (taskMap[task]) {
        console.log(`🎨 Executing ${task} task...`);
        await taskMap[task]();
      } else {
        console.warn(`⚠️  Unknown task: ${task}`);
      }
    }

    console.log('✅ Selected UI Enhancement tasks completed!');
  }
}

// Export the class
module.exports = UIEnhancementSkill;

// If running directly
if (require.main === module) {
  const skill = new UIEnhancementSkill(process.argv[2] || './frontend');

  if (process.argv[3] === '--tasks') {
    const tasks = process.argv.slice(4);
    skill.executeTasks(tasks).catch(console.error);
  } else {
    skill.executeAll().catch(console.error);
  }
}