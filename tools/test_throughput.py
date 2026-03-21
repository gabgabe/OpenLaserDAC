#!/usr/bin/env python3
"""
ILDAWaveX16 - Etherdream Performance Test
Test target: 80,000 points/sec throughput

Usage:
    ./test_throughput.py [target_pps] [batch_size]
    
Examples:
    ./test_throughput.py 80000 200    # 80 kpps with 200-point batches
    ./test_throughput.py 30000 70     # 30 kpps with 70-point batches (MadMapper)
"""

import socket
import struct
import time
import sys
import signal
from collections import deque

# Configuration
TARGET_IP = '192.168.77.1'
TARGET_PORT = 7765

# Test parameters (can override via command line)
TARGET_PPS = 80000      # Target points per second
BATCH_SIZE = 200        # Points per DATA command
TEST_DURATION = 60      # Test duration in seconds

# Success criteria
MAX_BUFFER_LEVEL = 6000   # Buffer should stay under this
MIN_BUFFER_LEVEL = 1000   # Buffer should stay above this (avoid underrun)
MAX_UNDERRUN_RATE = 1     # Max underruns per minute

class Colors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    RESET = '\033[0m'
    BOLD = '\033[1m'

def format_color(text, color):
    return f"{color}{text}{Colors.RESET}"

# Global state for clean exit
running = True
stats = {
    'batches_sent': 0,
    'points_sent': 0,
    'bytes_sent': 0,
    'acks_received': 0,
    'buffer_levels': deque(maxlen=100),
    'underruns': 0,
    'start_time': 0,
}

def signal_handler(sig, frame):
    global running
    print("\n\n" + format_color("Stopping test...", Colors.YELLOW))
    running = False

signal.signal(signal.SIGINT, signal_handler)

def parse_status(status_bytes):
    """Parse Etherdream status structure (20 bytes)"""
    # struct etherdream_status_t {
    #     uint8_t  protocol;            // 0
    #     uint8_t  light_engine_state;  // 1
    #     uint8_t  playback_state;      // 2
    #     uint8_t  source;              // 3
    #     uint16_t light_engine_flags;  // 4-5
    #     uint16_t playback_flags;      // 6-7
    #     uint16_t source_flags;        // 8-9
    #     uint16_t buffer_fullness;     // 10-11
    #     uint32_t point_rate;          // 12-15
    #     uint32_t point_count;         // 16-19
    # };
    
    if len(status_bytes) < 20:
        return None
    
    return {
        'protocol': status_bytes[0],
        'light_engine_state': status_bytes[1],
        'playback_state': status_bytes[2],
        'source': status_bytes[3],
        'buffer_fullness': struct.unpack('<H', status_bytes[10:12])[0],
        'point_rate': struct.unpack('<I', status_bytes[12:16])[0],
        'point_count': struct.unpack('<I', status_bytes[16:20])[0],
    }

def connect_etherdream(ip, port):
    """Connect to Etherdream server and prepare for streaming"""
    print(format_color(f"Connecting to {ip}:{port}...", Colors.CYAN))
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.settimeout(5.0)
    
    try:
        sock.connect((ip, port))
        print(format_color("✓ Connected", Colors.GREEN))
    except Exception as e:
        print(format_color(f"✗ Connection failed: {e}", Colors.RED))
        return None
    
    # Send PREPARE command
    print("Sending PREPARE command...")
    sock.send(b'p')
    
    # Receive ACK
    try:
        ack = sock.recv(22)  # response(1) + command(1) + status(20)
        if len(ack) < 22 or ack[0] != ord('a'):
            print(format_color("✗ PREPARE failed", Colors.RED))
            return None
        print(format_color("✓ PREPARE ACK", Colors.GREEN))
    except socket.timeout:
        print(format_color("✗ PREPARE timeout", Colors.RED))
        return None
    
    # Send BEGIN command (low_water=1700, rate=target_pps)
    print(f"Sending BEGIN command (rate={TARGET_PPS} pps, low_water=1700)...")
    begin_cmd = struct.pack('<cHI', b'b', 1700, TARGET_PPS)
    sock.send(begin_cmd)
    
    # Receive ACK
    try:
        ack = sock.recv(22)
        if len(ack) < 22 or ack[0] != ord('a'):
            print(format_color("✗ BEGIN failed", Colors.RED))
            return None
        
        status = parse_status(ack[2:22])
        print(format_color(f"✓ BEGIN ACK - Playback state: {status['playback_state']}", Colors.GREEN))
    except socket.timeout:
        print(format_color("✗ BEGIN timeout", Colors.RED))
        return None
    
    return sock

def send_batch(sock, batch_size):
    """Send one DATA command with batch_size points"""
    # DATA command: cmd(1) + npoints(2) + points(n×18)
    data_cmd = struct.pack('<BH', ord('d'), batch_size)
    
    # Generate test points (simple pattern)
    # etherdream_point_t: control(2) + x(2) + y(2) + r(2) + g(2) + b(2) + i(2) + u1(2) + u2(2)
    points = bytearray(batch_size * 18)
    for i in range(batch_size):
        offset = i * 18
        # Simple sine wave pattern
        angle = (i / batch_size) * 6.28318  # 2*pi
        x = int(32767 * 0.8 * (angle / 6.28318 - 0.5))
        y = int(32767 * 0.6 * ((i % 2) - 0.5))
        r = 65535 if i % 3 == 0 else 0
        g = 65535 if i % 3 == 1 else 0
        b = 65535 if i % 3 == 2 else 0
        
        struct.pack_into('<Hhhhhhhhhh', points, offset, 
                        0,      # control
                        x, y,   # position
                        r, g, b,  # color
                        65535,  # intensity
                        0, 0)   # user fields
    
    # Send command + points
    sock.sendall(data_cmd + points)
    
    # Update stats
    stats['batches_sent'] += 1
    stats['points_sent'] += batch_size
    stats['bytes_sent'] += len(data_cmd) + len(points)
    
    # Receive ACK
    try:
        ack = sock.recv(22)
        if len(ack) >= 22 and ack[0] == ord('a'):
            stats['acks_received'] += 1
            status = parse_status(ack[2:22])
            if status:
                stats['buffer_levels'].append(status['buffer_fullness'])
                return status
    except socket.timeout:
        pass
    
    return None

def print_stats_header():
    """Print statistics table header"""
    print("\n" + format_color("=" * 100, Colors.BLUE))
    print(format_color(
        f"{'Time':<8} {'PPS':<10} {'Batch/s':<10} {'Buffer':<15} {'Avg Buf':<10} "
        f"{'Latency':<10} {'Status':<12}",
        Colors.BOLD
    ))
    print(format_color("=" * 100, Colors.BLUE))

def print_stats_line(elapsed, last_status):
    """Print one line of statistics"""
    # Calculate rates
    actual_pps = stats['points_sent'] / elapsed if elapsed > 0 else 0
    batches_per_sec = stats['batches_sent'] / elapsed if elapsed > 0 else 0
    
    # Buffer statistics
    current_buffer = last_status['buffer_fullness'] if last_status else 0
    avg_buffer = sum(stats['buffer_levels']) / len(stats['buffer_levels']) if stats['buffer_levels'] else 0
    
    # Latency estimate (time per batch)
    avg_latency_ms = (elapsed * 1000) / stats['batches_sent'] if stats['batches_sent'] > 0 else 0
    
    # Status color coding
    if current_buffer < MIN_BUFFER_LEVEL:
        buf_color = Colors.RED
        status_text = "⚠ LOW BUF"
    elif current_buffer > MAX_BUFFER_LEVEL:
        buf_color = Colors.YELLOW
        status_text = "⚠ HIGH BUF"
    elif abs(actual_pps - TARGET_PPS) / TARGET_PPS > 0.1:
        buf_color = Colors.YELLOW
        status_text = "⚠ RATE OFF"
    else:
        buf_color = Colors.GREEN
        status_text = "✓ OK"
    
    print(f"{elapsed:7.1f}s {actual_pps:9.0f} {batches_per_sec:9.1f} "
          f"{format_color(f'{current_buffer:4d}/8192', buf_color):<24} {avg_buffer:9.1f} "
          f"{avg_latency_ms:9.2f}ms {format_color(status_text, buf_color)}")

def run_test(target_pps, batch_size, duration):
    """Run throughput test"""
    global running, TARGET_PPS, BATCH_SIZE
    TARGET_PPS = target_pps
    BATCH_SIZE = batch_size
    
    batches_per_sec = target_pps / batch_size
    interval = 1.0 / batches_per_sec
    
    print("\n" + format_color("=" * 60, Colors.CYAN))
    print(format_color(f"  ILDAWaveX16 Performance Test", Colors.BOLD))
    print(format_color("=" * 60, Colors.CYAN))
    print(f"  Target:        {format_color(f'{target_pps:,} pps', Colors.BOLD)}")
    print(f"  Batch size:    {format_color(f'{batch_size} points', Colors.BOLD)}")
    print(f"  Batch rate:    {format_color(f'{batches_per_sec:.1f} batch/sec', Colors.BOLD)}")
    print(f"  Interval:      {format_color(f'{interval*1000:.2f} ms', Colors.BOLD)}")
    print(f"  Duration:      {format_color(f'{duration} seconds', Colors.BOLD)}")
    print(format_color("=" * 60, Colors.CYAN))
    
    # Connect
    sock = connect_etherdream(TARGET_IP, TARGET_PORT)
    if not sock:
        return False
    
    print("\n" + format_color("Starting streaming test...", Colors.GREEN))
    print(format_color("Press Ctrl+C to stop early\n", Colors.YELLOW))
    
    stats['start_time'] = time.time()
    next_batch_time = stats['start_time']
    last_print_time = stats['start_time']
    last_status = None
    
    print_stats_header()
    
    try:
        while running:
            now = time.time()
            elapsed = now - stats['start_time']
            
            # Check test duration
            if elapsed >= duration:
                break
            
            # Send batch if it's time
            if now >= next_batch_time:
                last_status = send_batch(sock, batch_size)
                next_batch_time += interval
                
                # Drift correction: if we're behind, skip ahead
                if next_batch_time < now:
                    next_batch_time = now + interval
            
            # Print stats every second
            if now - last_print_time >= 1.0:
                print_stats_line(elapsed, last_status)
                last_print_time = now
            
            # Small sleep to avoid busy-wait
            sleep_time = min(interval / 10, next_batch_time - now)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except Exception as e:
        print(format_color(f"\n✗ Error: {e}", Colors.RED))
    
    finally:
        # Send STOP command
        print("\n" + format_color("Sending STOP command...", Colors.CYAN))
        try:
            sock.send(b's')
            sock.recv(22)  # ACK
        except:
            pass
        
        sock.close()
    
    # Print final results
    print_final_results()
    
    return True

def print_final_results():
    """Print final test results and success criteria"""
    elapsed = time.time() - stats['start_time']
    
    print("\n" + format_color("=" * 60, Colors.CYAN))
    print(format_color("  FINAL RESULTS", Colors.BOLD))
    print(format_color("=" * 60, Colors.CYAN))
    
    # Performance metrics
    actual_pps = stats['points_sent'] / elapsed if elapsed > 0 else 0
    actual_mbps = (stats['bytes_sent'] * 8) / (elapsed * 1_000_000) if elapsed > 0 else 0
    avg_buffer = sum(stats['buffer_levels']) / len(stats['buffer_levels']) if stats['buffer_levels'] else 0
    max_buffer = max(stats['buffer_levels']) if stats['buffer_levels'] else 0
    min_buffer = min(stats['buffer_levels']) if stats['buffer_levels'] else 0
    
    print(f"  Test duration:     {elapsed:.1f} seconds")
    print(f"  Batches sent:      {stats['batches_sent']:,}")
    print(f"  Points sent:       {stats['points_sent']:,}")
    print(f"  Actual PPS:        {format_color(f'{actual_pps:,.0f}', Colors.BOLD)}")
    print(f"  Network throughput: {actual_mbps:.2f} Mbps")
    print(f"  ACKs received:     {stats['acks_received']:,} ({100*stats['acks_received']/stats['batches_sent']:.1f}%)")
    print()
    print(f"  Buffer avg:        {avg_buffer:.0f} / 8192 ({100*avg_buffer/8192:.1f}%)")
    print(f"  Buffer range:      {min_buffer} - {max_buffer}")
    
    # Success criteria evaluation
    print("\n" + format_color("  SUCCESS CRITERIA", Colors.BOLD))
    print(format_color("  " + "-" * 56, Colors.BLUE))
    
    # Rate accuracy
    rate_error = abs(actual_pps - TARGET_PPS) / TARGET_PPS
    rate_ok = rate_error < 0.05  # ±5%
    rate_status = format_color("✓ PASS", Colors.GREEN) if rate_ok else format_color("✗ FAIL", Colors.RED)
    print(f"  Rate accuracy:     {rate_status}  ({100*(1-rate_error):.1f}% of target)")
    
    # Buffer stability
    buf_ok = MIN_BUFFER_LEVEL <= avg_buffer <= MAX_BUFFER_LEVEL
    buf_status = format_color("✓ PASS", Colors.GREEN) if buf_ok else format_color("✗ FAIL", Colors.RED)
    print(f"  Buffer stability:  {buf_status}  (avg {avg_buffer:.0f}, range {MIN_BUFFER_LEVEL}-{MAX_BUFFER_LEVEL})")
    
    # Overall
    all_ok = rate_ok and buf_ok
    if all_ok:
        print("\n" + format_color("  🎉 TEST PASSED! System meets 80 kpps target", Colors.GREEN + Colors.BOLD))
    else:
        print("\n" + format_color("  ⚠️  TEST FAILED - See issues above", Colors.RED + Colors.BOLD))
    
    print(format_color("=" * 60, Colors.CYAN))

if __name__ == '__main__':
    # Parse command line arguments
    if len(sys.argv) > 1:
        TARGET_PPS = int(sys.argv[1])
    if len(sys.argv) > 2:
        BATCH_SIZE = int(sys.argv[2])
    if len(sys.argv) > 3:
        TEST_DURATION = int(sys.argv[3])
    
    success = run_test(TARGET_PPS, BATCH_SIZE, TEST_DURATION)
    sys.exit(0 if success else 1)
