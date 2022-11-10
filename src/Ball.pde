public class SoccerBall {
  public Vec2 position;
  public float radius = 40;
  public Vec2 velocity;
  public boolean saved = false;
  public float rotation;
  
  
  public SoccerBall(float startX){
    this.position = new Vec2(startX, 480);
    this.velocity = new Vec2(random(-3, 3), random(-6, -3));
    //this.velocity = new Vec2(0, random(-6, -3));
    this.rotation = random(0,50);
  }
  public void update(){
    this.position.x += this.velocity.x;
    this.position.y += this.velocity.y;
  }
}
