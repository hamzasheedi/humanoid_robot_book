import time
import asyncio
import aiohttp
import statistics

async def measure_response_time(session, url, payload):
    """Measure response time for a single request"""
    start_time = time.time()
    try:
        async with session.post(url, json=payload) as response:
            await response.text()  # Consume the response
    except Exception as e:
        print(f"Error: {e}")
        return float('inf')
    
    end_time = time.time()
    return end_time - start_time

async def performance_test():
    """Performance testing: measure response times for cloud and local deployment"""
    url = "http://localhost:8000/chat/ask"  # Adjust this to your actual endpoint
    
    # Sample questions for testing
    test_questions = [
        {"question": "What is ROS 2?", "session_id": "test_session_1"},
        {"question": "Explain path planning algorithms", "session_id": "test_session_1"},
        {"question": "How does computer vision work in robotics?", "session_id": "test_session_1"},
        {"question": "What are the components of a humanoid robot?", "session_id": "test_session_1"},
        {"question": "Describe SLAM in robotics", "session_id": "test_session_1"}
    ]
    
    print("Starting performance tests...")
    
    async with aiohttp.ClientSession() as session:
        response_times = []
        
        for i, question_data in enumerate(test_questions):
            print(f"Test {i+1}/5: Measuring response time...")
            response_time = await measure_response_time(session, url, question_data)
            response_times.append(response_time)
            print(f"  Response time: {response_time:.3f}s")
    
    if response_times:
        avg_time = statistics.mean(response_times)
        min_time = min(response_times)
        max_time = max(response_times)
        
        print(f"\nPerformance Results:")
        print(f"Average response time: {avg_time:.3f}s")
        print(f"Min response time: {min_time:.3f}s")
        print(f"Max response time: {max_time:.3f}s")
        print(f"95th percentile: {sorted(response_times)[int(0.95 * len(response_times))]:.3f}s")
        
        # Check if requirements are met
        cloud_requirement = 2.0  # seconds
        local_requirement = 5.0  # seconds (for local deployment)
        
        if avg_time <= cloud_requirement:
            print(f"✅ Performance requirement met (≤{cloud_requirement}s) for cloud deployment")
        else:
            print(f"❌ Performance requirement not met (≤{cloud_requirement}s) for cloud deployment")
        
        return {
            "avg_response_time": avg_time,
            "min_response_time": min_time,
            "max_response_time": max_time,
            "tests_passed": avg_time <= cloud_requirement,
            "response_times": response_times
        }
    else:
        print("No valid response times measured")
        return None

if __name__ == "__main__":
    # Note: This test would require the backend server to be running
    print("Note: This test requires the backend server to be running on http://localhost:8000")
    results = asyncio.run(performance_test())
    if results:
        print(f"Performance test results: {results}")