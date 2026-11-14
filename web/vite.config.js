import {defineConfig} from 'vite';
export default defineConfig({base:process.env.PUBLIC_BASE_PATH||'/',server:{proxy:{'/api':'http://127.0.0.1:8000','/models':'http://127.0.0.1:8000'}}});
