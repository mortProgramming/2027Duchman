import java.net.URI;
import java.net.http.*;
import java.nio.ByteBuffer;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;

/**
 * Read-only dump from the Limelight web UI socket (ws://localhost:5805 through the rio tunnel).
 * Sends only "request_*" / "get_*" messages, the same ones the web UI sends to display settings,
 * and writes each distinct reply type to outDir/<type>.txt. Never sends set/upload/delete.
 */
public class LlDump {
  public static void main(String[] a) throws Exception {
    Path out = Paths.get(a[0]);
    Files.createDirectories(out);
    List<String> requests = Arrays.asList(Arrays.copyOfRange(a, 1, a.length));
    for (String r : requests) {
      if (!(r.startsWith("request_") || r.startsWith("get_"))) throw new IllegalArgumentException("read-only requests only: " + r);
    }
    Map<String, String> replies = new ConcurrentHashMap<>();
    StringBuilder sb = new StringBuilder();
    WebSocket ws = HttpClient.newHttpClient().newWebSocketBuilder()
        .subprotocols("soap", "xmpp")
        .buildAsync(URI.create("ws://localhost:5805"), new WebSocket.Listener() {
          public CompletionStage<?> onText(WebSocket w, CharSequence d, boolean last) {
            sb.append(d);
            if (last) {
              String s = sb.toString();
              sb.setLength(0);
              int sp = s.indexOf(' ');
              String type = sp > 0 ? s.substring(0, sp) : s;
              if (!type.equals("status_update")) System.out.println("reply: " + type + " (" + s.length() + " chars)");
              replies.put(type, s);
            }
            w.request(1);
            return null;
          }
          public CompletionStage<?> onBinary(WebSocket w, ByteBuffer d, boolean last) { w.request(1); return null; }
          public void onError(WebSocket w, Throwable e) { System.out.println("WS ERROR " + e); }
        }).get(8, TimeUnit.SECONDS);
    for (String r : requests) {
      ws.sendText(r + " 0*#", true).get();
      Thread.sleep(400);
    }
    Thread.sleep(3000);
    for (var e : replies.entrySet()) {
      Files.writeString(out.resolve(e.getKey() + ".txt"), e.getValue());
    }
    System.exit(0);
  }
}
