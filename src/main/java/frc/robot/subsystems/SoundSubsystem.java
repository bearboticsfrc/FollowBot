package frc.robot.subsystems;

import com.jcraft.jsch.ChannelExec;
import com.jcraft.jsch.JSch;
import com.jcraft.jsch.JSchException;
import com.jcraft.jsch.Session;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.List;

public class SoundSubsystem extends SubsystemBase {

  private static final String host = "photonvision.local";
  private static final String login = "pi";
  private static final String password = "raspberry";

  Session session = null;

  public SoundSubsystem() {

    try {
      session = setupSshSession();
      session.connect();

    } catch (JSchException e) {
      e.printStackTrace();
    }
  }

  public void playSound1() {
    Thread t = new Thread(() -> ssh("omxplayer PlayfulR2D2.mp3"));
    t.start();
  }

  public void playSound2() {
    new Thread(() -> ssh("omxplayer SadR2D2.mp3")).start();
  }

  public void playSound3() {
    new Thread(() -> ssh("omxplayer ProcessingR2D2.mp3")).start();
  }

  private List<String> ssh(String command) {
    ChannelExec channel = null;
    try {
      channel = (ChannelExec) session.openChannel("exec");
      channel.setCommand(command);
      channel.setInputStream(null);
      channel.setErrStream(System.err);
      InputStream output = channel.getInputStream();
      channel.connect();
      String result = resultToString(output);
      return Arrays.asList(result.split("\n"));
    } catch (JSchException | IOException ex) {
      System.out.println(ex);
      throw new RuntimeException(ex);
    } finally {
      channel.disconnect();
    }
  }

  private static String resultToString(InputStream inputStream) throws IOException {
    ByteArrayOutputStream result = new ByteArrayOutputStream();
    byte[] buffer = new byte[1024];
    int length;
    while ((length = inputStream.read(buffer)) != -1) {
      result.write(buffer, 0, length);
    }
    return result.toString(StandardCharsets.UTF_8);
  }

  private static Session setupSshSession() throws JSchException {
    Session session = new JSch().getSession(login, host, 22);
    session.setPassword(password);
    session.setConfig("PreferredAuthentications", "publickey,keyboard-interactive,password");
    session.setConfig("StrictHostKeyChecking", "no");
    return session;
  }

  private static void closeConnection(ChannelExec channel, Session session) {
    if (session == null || channel == null) return;
    try {
      if (channel != null && channel.isConnected()) {
        channel.disconnect();
      }
      if (session != null && session.isConnected()) {
        session.disconnect();
      }
    } catch (Exception ignored) {
    }
  }
}
