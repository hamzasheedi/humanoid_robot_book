def verify_no_keys_in_frontend():
    """Check that API keys are not exposed in frontend code"""
    import os
    
    print("Checking for API key exposure in frontend...")
    
    frontend_dir = "frontend"
    api_key_patterns = [
        "OPENAI_API_KEY",
        "QDRANT_API_KEY", 
        "NEON_DB_URL",
        "API_KEY"
    ]
    
    files_checked = 0
    issues_found = []
    
    for root, dirs, files in os.walk(frontend_dir):
        for file in files:
            if file.endswith(('.js', '.jsx', '.ts', '.tsx', '.html', '.vue')):
                filepath = os.path.join(root, file)
                try:
                    with open(filepath, 'r', encoding='utf-8') as f:
                        content = f.read()
                        
                        for pattern in api_key_patterns:
                            if pattern in content and 'process.env.' + pattern not in content:
                                issues_found.append(f"{filepath}: Contains '{pattern}' (possible exposure)")
                                
                    files_checked += 1
                except Exception as e:
                    print(f"Error reading {filepath}: {e}")
    
    print(f"Checked {files_checked} frontend files")
    
    if issues_found:
        print("❌ Potential API key exposures found:")
        for issue in issues_found:
            print(f"  - {issue}")
        return False
    else:
        print("✅ No API keys found exposed in frontend code")
        return True

def verify_secure_practices():
    """Verify other security best practices"""
    print("\nVerifying secure practices...")
    
    # Check for .env files in frontend (should not exist)
    import os
    env_found = False
    for root, dirs, files in os.walk("frontend"):
        for file in files:
            if file.startswith(".env"):
                print(f"❌ Found .env file in frontend: {os.path.join(root, file)}")
                env_found = True
                
    if not env_found:
        print("✅ No .env files found in frontend")
    
    # Verify backend uses proper security measures
    try:
        with open("backend/src/api/main.py", "r") as f:
            backend_main = f.read()
            
        has_cors = "CORSMiddleware" in backend_main
        has_secure_headers = "https://" in backend_main or "Secure" in backend_main
        
        if has_cors:
            print("✅ CORS middleware found in backend")
        else:
            print("⚠️  CORS configuration not found in backend")
            
    except FileNotFoundError:
        print("⚠️  Backend main.py not found to check security measures")
        
    return not env_found  # Pass if no .env files in frontend

def security_audit():
    """Run comprehensive security audit"""
    print("="*50)
    print("SECURITY AUDIT")
    print("="*50)
    
    key_exposure_ok = verify_no_keys_in_frontend()
    secure_practices_ok = verify_secure_practices()
    
    print("\n" + "="*50)
    if key_exposure_ok and secure_practices_ok:
        print("✅ SECURITY AUDIT PASSED: No obvious security issues found")
        return True
    else:
        print("❌ SECURITY AUDIT FAILED: Potential security issues detected")
        return False

if __name__ == "__main__":
    result = security_audit()
    exit(0 if result else 1)