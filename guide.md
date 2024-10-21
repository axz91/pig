## Edit the service file:

```bash {"id":"01J77N8YMK9JZR9WNEKXCWGDBB"}
sudo nano /etc/systemd/system/pig_robot.service

```

## Update the content of the file to match the following, making sure to use the correct path to your Python script:

```bash {"id":"01J77N8YMK9JZR9WNEKZZPT68M"}
sudo nano /etc/systemd/system/pig_robot.service

```

```bash {"id":"01J77N8YMK9JZR9WNEM21X381K"}
[Unit]
Description=Pig Robot Control Script
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/Desktop/ROBOTSOFTWARE/Untitled-1.py
WorkingDirectory=/home/pi/Desktop/ROBOTSOFTWARE
StandardOutput=inherit
StandardError=inherit
Restart=always
User=pi

[Install]
WantedBy=multi-user.target

```

3. Save the file and exit the editor (in nano, press Ctrl+X, then Y, then Enter).
4. Reload the systemd manager configuration:

```bash {"id":"01J77N8YMK9JZR9WNEM3MZ8G6V"}
sudo systemctl daemon-reload

```

5. Try to start the service:

```bash {"id":"01J77N8YMK9JZR9WNEM6ZAKHTN"}
sudo systemctl start pig_robot.service
sudo systemctl restart pig_robot.service
sudo systemctl disable --now pig_robot.service
```

6. Check the status of the service:

```bash {"id":"01J77N8YMK9JZR9WNEM7XMHGT4"}
sudo systemctl status pig_robot.service
journalctl -u pig_robot.service
sudo systemctl disable --now pig_robot.service
```

7. Stop the service:
